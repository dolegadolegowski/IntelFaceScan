# Reconstruction

This document covers how IntelFaceScan turns the guided-scan keyframes into a head mesh: segmentation,
registration, the pose graph, TSDF fusion on Metal with marching cubes, and screened Poisson as the comparison
method. It records the voxel size choice and the measurements behind it. The code lives in
`Sources/FaceReconCore/src/{registration,fusion,third_party/poisson_*}` (C API: `fr_registration.h`,
`fr_fusion.h`) and `Sources/ReconKit/{ReconstructionPipeline,Registration*,Segmentation*}.swift`.

Conventions follow `fr_types.h`. Units are meters. The camera frame is +X right, +Y down, +Z forward. Poses are
`world_from_camera`, and the **world is the camera frame of the first usable frontal keyframe** (the reference,
frame 0 of the registration).

## Pipeline (`ReconstructionPipeline.run`)

| stage (progress) | what happens | where |
|---|---|---|
| preparing | order keyframes front, up, down, left, right, diagonals (stable); drop frames with inconsistent image data | Swift |
| segmentation | head masks per keyframe (below); at least 5 usable keyframes including a frontal one, otherwise `notEnoughKeyframes` / `core(stage: "validation")` | `SegmentationHeadMask.swift` |
| registration | initial poses, `fr_register_frames` (clouds, pairwise ICP, loop closures) | `RegistrationInitialPoses.swift`, `fr_register_frames.cpp` |
| globalOptimization | pose graph (reported by the core's progress), then a registration check that drops frames whose surface does not fit | `fr_pose_graph.cpp`, `RegistrationCheck` |
| fusion | robust bounds of the registered clouds + margin, then TSDF integration on Metal (CPU fallback), or screened Poisson | `fr_tsdf.cpp`, `tsdf_metal.mm`, `fr_poisson.cpp` |
| meshing | marching cubes | `marching_cubes.cpp` |
| cleanup | crop below the chin unless `keepShoulders`, then `MeshProcessing.clean` | pipeline + mesh module |
| texturing | `Texturing.colorizeVertices`, `Texturing.unwrapAndBake` | mesh module |
| finished | export transform, report | pipeline |

The work runs on a background dispatch queue, never on the main actor or the cooperative thread pool. Task
cancellation sets a flag. The pipeline checks the flag between stages, and FaceReconCore's progress callbacks
return it, so long C++ calls stop early with `FR_ERR_CANCELLED` and the pipeline throws `.cancelled`. A failure
inside the mesh module's steps (cleanup, texturing) becomes a report note rather than an error, so the geometry is
not lost. `ReconstructionResult.cameraPoses` holds one pose per *used* keyframe, in the original capture order,
which is the same order passed to texturing. `report.usedFrames` gives the count, and the notes name any excluded
keyframes.

### Segmentation

The face depth is the median of valid depths in the inner half of the face rectangle. A pixel is a head candidate
when its depth lies in the band [face − 12 cm, face + 25 cm] and it is inside the keyframe's person mask. When the
keyframe has no mask, the candidate must instead lie in a generous region around the face rectangle. Of these
candidates, only the 4-connected component that overlaps the face rectangle most is kept, which removes background
objects at a similar depth.

**Chin cut.** Pixels farther than 8 % of the face height below the chin are dropped. The cut line is
perpendicular to the head's vertical axis in the image, rotated by the head roll. The chin is the lowest landmark
along that axis, or the bottom of the face rectangle when there are no landmarks.

Registration and the initial poses always use the cut mask, because the neck and shoulders do not rotate with the
head. Fusion uses the cut mask too, except with `keepShoulders`: the frontal keyframes then contribute the uncut
component, which includes the neck and shoulders. Only the frontal keyframes do this, because in the other views
the shoulders sit in a different place relative to the head and would smear.

**Roll from landmarks.** The landmark ordering is not relied on. The landmarks are leveled with the current roll
estimate, and the upper part of the face (excluding the center strip) is split into left and right halves. The
line between the centroids of the two halves is parallel to the eye line for any symmetric landmark
constellation. Three iterations converge. The estimate falls back to the Vision roll when there are fewer than 8
landmarks or the result is more than 15° away from Vision.

### Initial poses

`HeadPose.cameraFrameRotation` R_i is the head rotation in camera i. A head point p (world) therefore appears in
frame i at R_i R_ref^T (p − c_ref) + c_i, so world_from_camera_i has rotation R_ref R_i^T. The translation comes
from the first of these that succeeds:

1. **Landmark Kabsch.** Landmarks get 3D positions from the depth median within 2 px, inside the depth band. The
   frame's landmarks are fitted rigidly onto the reference's with Horn's closed form (quaternion from the dominant
   eigenvector of a 4×4 matrix, computed with Jacobi rotations), trimmed iteratively against landmarks Vision
   places on occluded parts of a turned face. The fit is accepted with at least 8 inliers, an RMS below 4 mm, a 3D
   spread above 6 mm, and a rotation within 12° of the Vision rotation. This fit also replaces the Vision rotation.
2. **Vision rotation plus landmark translation**: a trimmed mean of p_ref − R p_i over at least 6 landmark pairs.
3. **Vision rotation plus head centers.** Each head center is the centroid of the head-mask points pushed 4.5 cm
   back along the viewing ray, because the visible surface of a sphere has its centroid r/2 in front of the
   center.

Non-finite Vision angles count as 0. In the synthetic tests, every non-reference keyframe takes path 1.

## Point clouds (`fr_cloud_from_rgbd`)

`fr_cloud_from_rgbd` builds each cloud in these steps:

1. **Valid pixels**: depth in [min, max], and inside the mask (≥ 128).
2. **Silhouettes against flying pixels.** A valid pixel is "bad" when a 4-neighbor has no data or lies on another
   surface (|Δz| > max(5 mm, 2 · z/fx · tan 75°)); image-border pixels are bad too. The bad set is dilated by
   `mask_erode_px` with a separable square filter. Stereo depth interpolates between foreground and background
   along silhouettes, and those ramps disappear: there is a test with a 2-pixel ramp.
3. **Normals**: PCA over the neighbors within `normal_radius_px` (default 3 px, all offsets) that lie on the same
   surface. The tolerance grows with the pixel distance. Each normal is oriented toward the camera.
4. **Grazing samples** (angle between normal and ray > `max_view_angle_deg`) are dropped.
5. **Colors and intensity gradients.** Colors and intensity are taken from the image. The intensity gradient in
   the tangent plane (needed by colored ICP) is fitted by least squares in a 2D tangent basis over the same
   neighborhood.

Voxel downsampling (`fr_cloud_voxel_downsample`) sorts 63-bit voxel keys (21 bits per axis) and averages points,
normals (renormalized), colors, intensities and gradients (projected back to the tangent plane). The output is
deterministic.

## ICP (`fr_icp`)

Both variants run Gauss-Newton over a left perturbation T ← exp(ξ) T, with ξ = [ω, v] in the target frame:

* geometric: r_G = (Tq − p)·n_p, J_G = [q′ × n_p ; n_p]
* photometric (Park, Zhou, Koltun 2017): r_C = I_p + d_p·(Tq − p) − I_q − c, J_C = [q′ × d_p ; d_p]. Here d_p is
  the target's tangent-plane intensity gradient, and c is the mean intensity offset, which absorbs auto-exposure
  changes between views. The two terms are weighted λ and 1 − λ (λ = 0.968).

**Multi-scale.** The pyramid has `levels` levels with voxel · 2^l. Each level averages the *raw* points, so coarse
positions are unbiased. The correspondence distance and the robust scale grow by 2^l per level. Correspondences
come from nanoflann (k-d tree, radius-limited 1-NN).

**Robust kernel.** The kernel is Tukey (default) or Huber on r_G. A 1e-9 · trace/6 ridge keeps directions the data
does not constrain at zero update. Sliding on a plane is the textbook case: point-to-plane leaves the in-plane
shift untouched, and colored ICP resolves it (test).

**Colored ICP uses the photometric term only at the finest level.** With the term at the coarse levels, texture
created local minima: edges with 6° / 10 mm initial error were rejected. Geometric coarse levels plus photometric
full resolution keep sub-millimeter results up to 10° / 20 mm (table below).

**Result.** Fitness is inliers / source points at the finest level. The inlier RMSE is the point-to-plane
residual. The information matrix is computed as in Open3D: Σ GᵀG with G = [−[p]× | I] at the target points, so
information(5,5) is the correspondence count. `max_iterations = 0` only evaluates the initial alignment; the
pipeline's registration check uses this.

## Whole-scan registration (`fr_register_frames`)

**Pose groups.** The C API carries no pose labels. Consecutive frames whose initial rotations differ by less than
12° form a group, and the frontal group is the one containing frame 0.

**Candidate edges.** Every candidate is refined by ICP from the initial relative pose:

* **odometry**: consecutive frames of a group (certain). Within a group the head is held still, so ICP also tries
  the identity rotation (centroids aligned) and keeps the better result.
* **bridges**: each group to the frontal group, using the 3 frame pairs with the most similar initial rotations.
* **loop closures**: every other pair whose initial rotations differ by less than `loop_closure_max_angle_deg`
  (70°), at most 6 per frame, in order of increasing angle. Within a group, only skip-one pairs are used.

**Rejection.** An edge is rejected when its fitness is below `min_fitness`, its RMSE is above `max_rmse_m`, or ICP
moved far from the initial guess: more than 30° of rotation or more than 4 cm displacement of the source cloud's
centroid. The displacement is measured at the centroid, not as camera translation, because a few degrees about the
head 0.6 m in front of the camera already mean centimeters of camera translation. Finally, every edge fitting worse
than 1.5× the median RMSE is rejected. A sliding partial overlap can pass the absolute thresholds but still fits
clearly worse than the typical edge, whose RMSE reflects the depth noise.

**Certain spanning structure.** The certain edges are the accepted odometry edges plus, for each group, the
accepted bridge that fits *best after ICP* (fitness / RMSE). An earlier version took the bridge with the closest
initial rotation, and one of those slid into a wrong minimum that still passed the thresholds, which put a frame
83 mm off. Groups or frames that stay disconnected are attached through the best accepted alternative (bridges
first). Frames that cannot be connected at all keep their initial pose; the pipeline's registration check then
removes them from fusion. Initial node poses are composed along the certain edges.

**Pose graph.** Implementation details are in the next section. Frame 0 is fixed (`reference_node` is forced to
0). Certain edges are odometry edges and uncertain edges are loop closures. The report counts certain edges as
`odometry_edges` and surviving uncertain edges as `loop_closures`. Rejected and pruned edges are counted together.

**Diagnostics.** Set `FR_DEBUG_REGISTRATION=1` to print every candidate edge (kind, initial angle, status,
fitness, RMSE, acceptance) to stderr.

## Pose graph (`fr_pose_graph_*`)

The optimizer is Levenberg-Marquardt over left perturbations of world_from_node, with sparse LDLT (Eigen), the
Marquardt diagonal, and the gauge fixed at the reference node. Nodes not connected to the reference keep their
poses, so the system stays nonsingular.

**Residual.** The edge residual is defined by T_t⁻¹ T_s = exp(e) Z: it is a left perturbation of the measurement
in the target frame, which is the frame ICP computes its information in. The Jacobians are
J_s = J_l⁻¹(e) Ad(T_t⁻¹) and J_t = −J_s, with J_l⁻¹ ≈ I − ½ ad(e).

**Line processes (Choi, Zhou, Koltun 2015).** Each uncertain edge carries l = (μ / (μ + eᵀΛe))², the closed-form
optimum of l·eᵀΛe + μ(√l − 1)². This turns the edge's cost into μ·eᵀΛe / (μ + eᵀΛe). The weights are updated every
iteration.

**μ per edge.** μ = `loop_closure_preference` · (1 cm)² · (mean translational information). With ICP information
matrices (translational block = correspondence count · I) and the default preference 0.1, an edge whose
correspondences disagree with the optimized poses by more than √0.1 cm ≈ 3.2 mm RMS gets l < 0.25. It is then
pruned (`edge_prune_threshold`), and the graph is optimized again without it. That threshold fits the
registration's star topology (groups bridge directly to the frontal group), where drift stays well below 1 mm.

**Tests.** From a start 6° / 2 cm off, exact measurements are recovered to < 0.002° and 2 µm. A noisy 12-node loop
with three correct loop closures and one 5 cm outlier prunes exactly the outlier and more than halves the drift.

## TSDF fusion (`fr_tsdf_*`, `fr_fuse_frames`)

**Volume.** A dense grid over the given bounds (dimensions ⌈extent / voxel⌉ + 1). Each voxel holds half tsdf ∈
[−1, 1] (signed distance / truncation, positive in front of the surface) and half weight, interleaved, plus
optional RGBA8 color, where A counts color observations. That is 8 bytes per voxel. On the GPU path the volume
lives in **shared-storage `MTLBuffer`s** (unified memory), so marching cubes and the CPU read it in place, with no
copies.

**Size checks.** A volume is refused with a clear error when an axis would exceed 16384 voxels, the total would
exceed 1.5 G voxels (32-bit indices in the shader), or it would need more than min(½ physical memory, 32 GiB). The
error reads, for example: *"TSDF volume of 4001x4001x4001 voxels (477 GiB) exceeds the limit … increase
voxel_size_m or shrink the bounds"*. Before reaching that point, the pipeline coarsens the voxel when the volume
would exceed 600 M voxels, and adds a note.

**Per frame (CPU, shared by both paths).** Depth is converted to meters. The mask and the depth range are applied
(masked pixels never update voxels, so the background is neither fused nor carved). Then the observation weight is
computed: `weight_mode` 1 gives w = frame weight · cos(view angle) · (0.6 m / z)². The view angle comes from
depth-image normals with 3-pixel tangents (one-sided at edges); 1-pixel normals are useless under D455 noise. The
weight is normalized so that a frontal view at 0.6 m weighs about 1.

**Per voxel (Metal kernel or CPU loop, identical arithmetic).** Each voxel is projected into the frame. Depth is
interpolated bilinearly when the 4 neighbors lie on one surface, because voxels are smaller than the 0.94 mm pixel
footprint; otherwise the nearest pixel is used. Then:

* sdf = d − z; voxels with sdf < −truncation are skipped;
* tsdf = min(1, sdf / truncation);
* a running weighted average with the weight capped at `max_weight`;
* color is averaged with a count where |sdf| < truncation.

**Metal.** The shader is compiled at runtime from a C++ raw string with `newLibraryWithSource`, because the
Command Line Tools have no offline `metal` compiler. Device, library and pipeline state are created once per
process. The math mode is *safe*, so GPU and CPU agree to rounding (test: maximum vertex distance below 0.05 mm,
vertex counts within 0.1 %). The volume is cleared with a blit fill. One command buffer runs per frame and is
checked for errors. `tsdf_metal.mm` requires ARC, which SwiftPM enables for Objective-C++.

**Fallback.** Without a Metal device, or when the shader or buffers fail, the same volume layout is allocated
with `calloc` and integrated multi-threaded, with Grand Central Dispatch over z slices. `use_gpu = 0` forces this
path, and `fr_tsdf_uses_gpu` reports which one ran.

### Marching cubes

Marching cubes runs in parallel over z slabs (Grand Central Dispatch).

**Shared vertices.** Each voxel owns the three cube edges leaving its corner (+x, +y, +z), so every crossed edge
gets exactly one vertex. Vertex ids come from per-slice counts (first pass) and per-slab id layers rebuilt
deterministically (second pass), so the output does not depend on the thread count.

**Generated triangle table.** The table is generated at startup instead of being transcribed. On each cube face,
contour segments cut off the runs of inside corners: "inside corners separated" on ambiguous faces. The rule
depends only on the face's four corner signs, so neighboring cubes agree and the surface is crack-free. Segments
are oriented by walking each face counter-clockwise as seen from outside. They are chained into loops and
triangulated **without diagonals inside a cube face**. A plain fan could put a diagonal, or a whole triangle, into
an ambiguous face that the neighboring cube triangulates too, and noisy scans then had edges shared by 3–4
triangles. All 256 configurations triangulate with at most 5 triangles. Triangles are counter-clockwise seen from
outside (the positive side).

**Vertex attributes.** Normals come from the TSDF gradient (central differences, one-sided next to unobserved
voxels), interpolated along the edge. Colors are interpolated along the edge.

**Rejected crossings.** A cube takes part only when all 8 corners have weight ≥ `min_weight`; the pipeline uses
0.25, a quarter of a frontal observation. A sign change between values more than 1.0 apart (in truncation units)
is not a surface. It is a silhouette artifact, where one view's free space meets another view's behind-surface
band, so no vertex is made there and the cube leaves a hole instead of a flap.

**Tests.** A sphere seen from 26 directions gives a watertight mesh (every directed edge exactly once, Euler
characteristic 2) with radius bias and RMS below 0.3 voxel and outward gradient normals. A heavily noisy sphere
gives a manifold, consistently oriented mesh. This last test fails with the old fan triangulation.

## Screened Poisson (`fr_poisson_reconstruct`)

`third_party/PoissonRecon` (version 18.76, MIT) compiles as a header library through one unity file,
`src/third_party/poisson_recon.cpp`. It builds in about 7 s with `-O2`; its warnings are silenced only there. The
wrapper works as follows:

* **Input and output.** Oriented points, optionally with color as auxiliary data (jointly extrapolated). The
  output is triangles with per-vertex density, already counter-clockwise from outside.
* **Density trimming.** The `density_trim_quantile` lowest-density vertices are removed with their triangles.
  These are where Poisson closes every hole with a smooth, invented membrane. Normals are area-weighted.
* **Threading.** PoissonRecon keeps global thread-pool state, so calls are serialized. Its `ASYNC` thread pool
  races on a static futures vector when `ParallelFor` nests inside the `ParallelSections` of the level-set
  extraction: "Failed to close loop" exceptions, reproduced outside the app. The solver therefore runs
  multi-threaded and the extraction single-threaded. `threads = 1` makes everything serial; any other value uses
  all cores, because the pool size is fixed.

The pipeline feeds it all used frames' fusion-mask clouds in the world frame, voxel-averaged at `voxelSizeMM`,
with `poissonDepth`.

## TSDF versus Poisson

The comparison uses a synthetic scan the size of a real one. It has 40 keyframes at 1280×800 with the D455 color
geometry (90° horizontal field of view), 8 frontal frames and 4 each for up, down, left, right and the four
diagonals. The poses carry 1.5° hold jitter and 2.5° Vision pose error. The depth noise is σ = 0.0025 · z², which
is 0.9 mm at 0.6 m, quantized to 0.1 mm. There is a static neck and a back wall. The run is the full pipeline,
release build, and the reconstructions share the same registration.

**Error metrics.** The surface error is the distance along each vertex normal to the true head (an ellipsoid
union). "Nose" is the mean error within 2 cm of the nose tip, the finest detail in the model. Boundary edges count
holes and open borders.

| method | setting | total (s) | surface stage (s) | vertices | mean error (mm) | p95 (mm) | nose (mm) | boundary edges |
|---|---|---|---|---|---|---|---|---|
| TSDF (Metal) | voxel 1.00 mm | 1.06 | 0.09 | 121722 | 0.184 | 0.329 | 0.103 | 11640 |
| **TSDF (Metal)** | **voxel 0.75 mm** | **1.09** | **0.11** | **229962** | **0.186** | **0.331** | **0.090** | 16979 |
| TSDF (Metal) | voxel 0.50 mm | 1.18 | 0.20 | 551078 | 0.202 | 0.341 | 0.091 | 29241 |
| TSDF (CPU) | voxel 0.75 mm | 1.41 | 0.43 | 229962 | 0.186 | 0.331 | 0.090 | 16979 |
| Poisson | depth 9 | 11.58 | 10.60 | 562413 | 0.341 | 0.628 | 0.179 | 4972 |
| Poisson | depth 10 | 12.21 | 11.26 | 594441 | 0.406 | 1.116 | 0.318 | 4806 |
| Poisson | depth 11 | 12.91 | 11.92 | 589441 | 0.403 | 1.095 | 0.334 | 4803 |

**Decision: TSDF is the default method; Poisson stays available as `.poisson`.**

* **Accuracy.** TSDF is about 2× more accurate (mean 0.19 versus 0.34–0.41 mm, p95 0.33 versus 0.63–1.1 mm) and
  keeps detail better: nose error 0.09 versus 0.18–0.33 mm. Poisson fits the raw noisy points with an implicit
  function, while the TSDF averages every observation along the viewing rays with noise-aware weights. Poisson
  depths 10–11 (cells about 0.3 mm over the padded cube) fit the noise and get worse than depth 9.
* **Time.** TSDF is about 100× faster in the surface stage (0.1 s versus 11–12 s), and the whole pipeline runs in
  about 1.1 s.
* **Holes.** Poisson closes holes and gives a watertight surface before trimming. The TSDF leaves unobserved
  regions open. In this scan almost all TSDF boundary edges are in such regions:
  * the back of the head, which the guided-scan poses (|yaw| ≤ 45°) never see: one loop of about 5600 vertices;
  * the neck cut and the underside of the jaw;
  * self-occluded creases under the brow ridge and beside the nose;
  * observation fringes with low weight, about 350 tiny holes of 8 or fewer edges;
  * about 690 small noise islands, all under 100 vertices.

  The islands and tiny holes are what `MeshProcessing.clean` removes and fills (small components, holes up to
  `maxHoleEdges`). The large back-of-head opening is a coverage limit of the scan poses, not of the fusion: closing
  it needs views from behind, or hole filling over a large region.

## Voxel size and truncation

**Noise model.** D455 depth noise grows as σ_z = z² · ε_d / (f · b), with baseline b = 95 mm, f ≈ 674 px for the
1280×720 depth stream (87° horizontal field of view), and subpixel disparity error ε_d ≈ 0.08–0.25. At 0.6 m this
gives σ_z ≈ 0.45–1.4 mm before filtering, which matches the 0.5–1.5 mm expected for faces.

**Lateral resolution.** The color pixel footprint at 0.6 m is 0.6 / 640 = 0.94 mm, and the aligned depth has the
same sampling. Fusing many slightly different views recovers some sub-pixel detail, but surface features under
about 1 mm are not resolved.

**Fusion averaging.** Each surface point is seen by roughly 5–15 frames with useful weight. With independent noise
the fused surface noise is σ / √N_eff ≈ 0.15–0.5 mm. It is less in practice, because the temporal filter and the
static projector pattern correlate frames of one pose.

**Voxel.** Marching cubes places the surface with sub-voxel accuracy by interpolation, but represents features of
roughly 2 voxels and larger. A voxel of about half the raw noise (and close to the fused noise) keeps the D455's
real detail without turning noise into geometry. Measured on the synthetic scan (same setup as above, truncation
max(4 mm, 3σ)):

| depth noise at 0.6 m | voxel | mean error (mm) | p95 (mm) | nose (mm) | vertices |
|---|---|---|---|---|---|
| 0.5 mm | 1.00 mm | 0.143 | 0.242 | 0.083 | 118275 |
| 0.5 mm | 0.75 mm | 0.140 | 0.240 | 0.072 | 221541 |
| 0.5 mm | 0.50 mm | 0.151 | 0.246 | 0.067 | 525184 |
| 0.9 mm | 1.00 mm | 0.184 | 0.329 | 0.103 | 121722 |
| 0.9 mm | 0.75 mm | 0.186 | 0.331 | 0.090 | 229962 |
| 0.9 mm | 0.50 mm | 0.202 | 0.341 | 0.091 | 551078 |
| 1.5 mm | 1.00 mm | 0.266 | 0.512 | 0.164 | 130355 |
| 1.5 mm | 0.75 mm | 0.275 | 0.523 | 0.155 | 247310 |
| 1.5 mm | 0.50 mm | 0.298 | 0.556 | 0.155 | 599419 |

The overall error is set by the noise. Smaller voxels sharpen detail (nose) only while the noise is low, and add
noise everywhere else at 2.4× the vertices.

**Choice: 0.75 mm** (`ReconstructionOptions.voxelSizeMM` default).

* Use 0.5 mm only when the measured single-frame noise is ≤ 0.5 mm, for example after strong temporal filtering
  or at a closer working distance.
* Use 1.0 mm for noise ≥ 1.3 mm.

**Truncation.** Use 4 mm, which is ≥ 5 voxels and ≥ 3σ for σ ≤ 1.3 mm. Raise it to 3σ for noisier data; the
pipeline enforces at least 2 voxels. The synthetic data has independent per-pixel noise, while real D455 noise is
spatially correlated (spatial filter, projector pattern). The choice should be re-checked against real recordings
once the capture module's measurements (`docs/CAMERA.md`) are in.

**Memory.** A head volume with margins is about 0.27 × 0.33 × 0.30 m. That is about 60 M voxels (0.5 GB) at
0.75 mm and about 200 M voxels (1.6 GB) at 0.5 mm.

## Registration accuracy and capture range

**Whole-scan registration.** The test uses 11 rendered views of the synthetic head at 640×400: front ×3, up ×2,
down ×2, left ×2, right ×2. Depth noise is 0.9 mm at 0.6 m, and the initial poses are perturbed by 4° about the
head plus 6 mm. Every frame lands within **0.37 mm** of the truth (the worst displacement of head points), with
rotation error ≤ 0.13°. Edge RMSE is 0.6 mm, which is the noise level.

**Capture range.** Same views, 3 random perturbations per row; time is per registration:

| initial rotation error | initial translation error | colored ICP | worst misalignment (mm) | max rejected edges | time (s) |
|---|---|---|---|---|---|
| 2° | 3 mm | yes | 0.41 | 0 | 0.11 |
| 2° | 3 mm | no | 0.72 | 0 | 0.06 |
| 4° | 6 mm | yes | 0.42 | 0 | 0.12 |
| 4° | 6 mm | no | 0.71 | 0 | 0.06 |
| 6° | 10 mm | yes | 0.42 | 0 | 0.10 |
| 6° | 10 mm | no | 0.68 | 0 | 0.06 |
| 8° | 15 mm | yes | 0.54 | 5 | 0.11 |
| 8° | 15 mm | no | 0.75 | 5 | 0.08 |
| 10° | 20 mm | yes | 0.50 | 16 | 0.13 |
| 10° | 20 mm | no | 0.84 | 16 | 0.09 |

The perturbations are per frame, so relative errors between two frames are up to twice these values. Colored ICP
is about 40 % more accurate on the smooth synthetic head, where geometry alone leaves sliding directions.

**40-keyframe scan.** In the 1280×800 benchmark scan, the worst frame is 0.75 mm off (the left group) and the
median is about 0.25 mm.

## Timings

Measured on an Apple M3 Ultra (28 CPU cores, 96 GB) in a release build: the full pipeline on 40 keyframes at
1280×800 with 0.75 mm voxels on Metal. The task's reference machine is an M4 Pro; this one is somewhat faster on
GPU and CPU throughput.

| stage | seconds |
|---|---|
| segmentation (40 frames, concurrent) | 0.014 |
| registration (clouds, 173 ICP edges, pose graph) | 0.69 |
| registration check | 0.15 |
| bounds | 0.11 |
| fusion (TSDF, Metal; CPU path 0.43 s) | 0.15 |
| marching cubes (included in fusion) | – |
| crop | 0.005 |
| **total** | **1.14** |

This is far inside the 60 s target. Cleanup, texturing and export (mesh module) come on top; they were
placeholders in this branch. The debug-build unit tests run the 13-keyframe 640×400 pipeline in 1–3 s.

## Known limits

* **The back of the head is not observed** by the guided-scan poses (|yaw| ≤ 45°), so the TSDF mesh is open there.
  A closed model needs back or over-the-shoulder views from the scan module, or large-hole filling.
* **Frames that cannot be registered** keep their initial pose inside `fr_register_frames`. The C API has no
  per-frame status, so the pipeline re-scores every frame against its most similar neighbor
  (`fr_icp` with `max_iterations = 0`) and leaves failing frames out of fusion.
* **Export orientation.** Yaw and pitch come from the mean of all keyframes' Vision head poses carried into the
  world (outliers more than 10° off ignored). Roll comes from the reference eye line. A systematic Vision yaw bias
  for a person would tilt the export frame by that bias. A symmetry-plane fit on the mesh would remove it.
* **Chin crop fallback.** While `fr_mesh_crop_plane` is not implemented, the pipeline drops whole triangles below
  the plane (jagged border) and notes it.
* **Vertex colors from the TSDF** are count-averaged in RGBA8. They are a fallback: the texture and vertex colors
  of the mesh module replace them.
