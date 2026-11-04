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
| globalOptimization | pose graph (reported by the core's progress), then a registration check that drops frames whose surface does not fit (below) | `fr_pose_graph.cpp`, `RegistrationCheck` |
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
2. **Silhouettes against flying pixels.** A valid pixel is "bad" when a 4-neighbor lies on another surface
   (|Δz| > max(5 mm, 2 · z/fx · tan 75°)) or belongs to an empty region (no data or masked out) of more than 4
   pixels; image-border pixels and those empty regions are bad too. The bad set is dilated by `mask_erode_px` with a
   separable square filter. Stereo depth interpolates between foreground and background along silhouettes, and
   those ramps disappear: there is a test with a 2-pixel ramp. Smaller empty regions are matching failures inside a
   surface (speckle), not silhouettes. An earlier version eroded around them too, so every speckle erased a 5 × 5
   block: the synthetic default of 1 % speckle cost 38 % of a head cloud, and with 8 % scattered holes 98 % of the
   points were gone and the reconstruction failed (test with 8 % holes).
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
* photometric (Park, Zhou, Koltun 2017): r_C = I_p + d_p·(Tq − p) − g·I_q, J_C = [q′ × d_p ; d_p]. Here d_p is
  the target's tangent-plane intensity gradient, and g is the exposure gain between the views: the median ratio
  (I_p + d_p·(Tq − p)) / I_q over the current correspondences, so auto exposure (which scales intensities) does
  not bias the term. The two terms are weighted λ and 1 − λ (λ = 0.968), with intensities in [0, 1].

On the colored level every iteration first finds the correspondences, then estimates g and the photometric kernel
scale from them, then accumulates the normal equations.

**Multi-scale.** The pyramid has `levels` levels with voxel · 2^l. Each level averages the *raw* points, so coarse
positions are unbiased. The correspondence distance and the robust scale grow by 2^l per level. Correspondences
come from nanoflann (k-d tree, radius-limited 1-NN).

**Robust kernels.** Each residual has its own: Tukey (default) or Huber on r_G with `robust_scale_m`, and Tukey on
r_C with 4.685 · 1.4826 · MAD of the photometric residuals, clamped to [0.02, 0.25]. The photometric kernel matters:
on texture edges (eyes, brows, lips, hairline, freckles) the linear intensity model fails, and those ~2 % of the
correspondences have residuals 10–50× the color noise. Unweighted, they dominated the normal equations; see
"Colored versus point-to-plane ICP". The upper clamp keeps the kernel wide while the views are still misaligned,
so strong textures keep their capture range (the textured-plane test starts 3.6 mm off). A 1e-9 · trace/6 ridge
keeps directions the data does not constrain at zero update. Sliding on a plane is the textbook case:
point-to-plane leaves the in-plane shift untouched, and colored ICP resolves it (test).

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

**Registration check (`RegistrationCheck.verify`, Swift).** The registered poses are grouped (relative rotation
below 12° to a group's first frame). Every frame's registered cloud (stride 2, 1.5 mm voxels) is evaluated against
the most similar view (smallest relative rotation) of its own pose group and of every other pose group within 60°,
with `fr_icp` at `max_iterations = 0` and a 3 mm correspondence distance. A pair fits when its fitness is at least
0.25 and its RMSE at most max(2 × median, median + 0.7 mm) over all evaluated pairs. A frame passes when a chain of
fitting pairs links it to frame 0, which always passes. An earlier version judged each frame by its single most
similar view, which failed both ways: a pose group misregistered as a whole (a wrong bridge) fits itself, because
its frames are each other's most similar views, and passed; a correct frame whose most similar view was the
misregistered one was dropped (tests with a 3.6 mm offset of one frame and of a two-frame group). Comparing with a
fixed number of most similar views is not enough either: with four keyframes per pose those are all siblings, and
the left groups of the 40-keyframe benchmark were never linked to the reference (test with four views per pose).

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
* **Sanitized build.** `SANITIZED_PR` is defined: the default build reads shared node indices through volatile
  references (double-checked locking), which ThreadSanitizer reports as races; the sanitized variant uses atomics
  at no measurable cost.
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

| method | setting | surface stage (s) | vertices | mean error (mm) | p95 (mm) | nose (mm) |
|---|---|---|---|---|---|---|
| TSDF (Metal) | voxel 1.00 mm | 0.10 | 147476 | 0.214 | 0.360 | 0.076 |
| **TSDF (Metal)** | **voxel 0.75 mm** | **0.11** | **269319** | **0.196** | **0.291** | **0.060** |
| TSDF (Metal) | voxel 0.50 mm | 0.20 | 657268 | 0.213 | 0.285 | 0.068 |
| TSDF (CPU) | voxel 0.75 mm | 0.40 | 269319 | 0.196 | 0.291 | 0.060 |
| Poisson | depth 9 | 13.0 | 758803 | 0.325 | 0.591 | 0.155 |
| Poisson | depth 10 | 12.4 | 854738 | 0.354 | 0.925 | 0.276 |
| Poisson | depth 11 | 13.4 | 844951 | 0.360 | 0.943 | 0.275 |

Measured after integration (`IFS_BENCH=1 scripts/test.sh -c release --filter PipelineBenchmark`), so the meshes are
cleaned and textured; vertex counts include the copies along texture seams, which is also why boundary edges are no
longer a useful hole count here. The pre-integration measurement (no cleanup) had the same ranking: TSDF 0.75 mm
mean 0.186 / p95 0.331 mm, Poisson depth 9–11 0.344–0.410 / 0.64–1.13 mm.

**Decision: TSDF is the default method; Poisson stays available as `.poisson`.**

* **Accuracy.** TSDF is about 2× more accurate (mean 0.20 versus 0.33–0.36 mm, p95 0.29 versus 0.59–0.94 mm) and
  keeps detail better: nose error 0.06 versus 0.16–0.28 mm. Poisson fits the raw noisy points with an implicit
  function, while the TSDF averages every observation along the viewing rays with noise-aware weights. Poisson
  depths 10–11 (cells about 0.3 mm over the padded cube) fit the noise and get worse than depth 9.
* **Time.** TSDF is about 100× faster in the surface stage (0.1 s versus 12–13 s).
* **Holes.** Poisson closes holes and gives a watertight surface before trimming. The TSDF leaves unobserved
  regions open. In this scan (pre-integration, before cleanup) almost all TSDF boundary edges were in such regions:
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
the fused surface noise is σ / √N_eff ≈ 0.15–0.5 mm. The gain is smaller in practice, because the temporal filter
and the static projector pattern correlate frames of one pose.

**Voxel.** Marching cubes places the surface with sub-voxel accuracy by interpolation, but represents features of
roughly 2 voxels and larger. A voxel of about half the raw noise (and close to the fused noise) keeps the D455's
real detail without turning noise into geometry. Measured on the synthetic scan (same setup as above, truncation
max(4 mm, 3σ)):

| depth noise at 0.6 m | voxel | mean error (mm) | p95 (mm) | nose (mm) | vertices |
|---|---|---|---|---|---|
| 0.5 mm | 1.00 mm | 0.188 | 0.289 | 0.074 | 142905 |
| 0.5 mm | 0.75 mm | 0.170 | 0.246 | 0.069 | 256701 |
| 0.5 mm | 0.50 mm | 0.171 | 0.231 | 0.059 | 602498 |
| 0.9 mm | 1.00 mm | 0.214 | 0.360 | 0.076 | 147476 |
| 0.9 mm | 0.75 mm | 0.196 | 0.291 | 0.060 | 269319 |
| 0.9 mm | 0.50 mm | 0.213 | 0.285 | 0.068 | 657268 |
| 1.5 mm | 1.00 mm | 0.279 | 0.468 | 0.075 | 164477 |
| 1.5 mm | 0.75 mm | 0.274 | 0.433 | 0.072 | 308163 |
| 1.5 mm | 0.50 mm | 0.290 | 0.462 | 0.080 | 778462 |

(After integration, cleaned and textured meshes; `PipelineBenchmark.voxelSizeVersusNoise`.) The overall error is set
by the noise. 0.75 mm is the best or within 1 % of the best at every noise level; 0.5 mm sharpens the nose only at
the lowest noise and costs 2.4× the vertices. Before integration (no cleanup) 1.0 mm was slightly better at 1.5 mm
noise (0.266 versus 0.275 mm); the cleanup's smoothing removed that difference.

**Choice: 0.75 mm** (`ReconstructionOptions.voxelSizeMM` default).

* 0.5 mm brings nothing measurable, even at 0.5 mm noise or at 0.47 m with correlated noise (below).
* 1.0 mm is a reasonable choice to save memory and time; it costs 2–11 % accuracy.

**Truncation.** Use 4 mm, which is ≥ 5 voxels and ≥ 3σ for σ ≤ 1.3 mm. Raise it to 3σ for noisier data; the
pipeline enforces at least 2 voxels, also after coarsening a too-large volume.

**Check against the D455 numbers with correlated noise.** The table above has independent per-pixel noise, while
real D455 noise is spatially correlated (spatial filter, projector pattern). Textured `SyntheticData` head, full
pipeline (point-to-plane), 10 keyframes at 1280×800. At the recommended 0.47 m with the theoretical white noise
of `docs/CAMERA.md` (1.25 mm at 1 m: 0.28 mm at 0.47 m), and at 0.6 m with the renderer's default (2 mm at 1 m:
0.72 mm), both with correlated noise (σ 1.4 mm at 1 m, correlation radius 4 px) and waviness (σ 0.8 mm at 1 m,
24 px) on top (`PipelineRobustnessTests.tsdfParameterTable`, `IFS_RECON_STUDY=1`):

| scan | voxel | truncation | surface mean / p95 |
|---|---|---|---|
| 0.47 m | 0.50 mm | 2.5 mm | 0.193 / 0.592 mm |
| 0.47 m | 0.50 mm | 4.0 mm | 0.188 / 0.576 mm |
| 0.47 m | 0.75 mm | 2.5 mm | 0.184 / 0.574 mm |
| 0.47 m | **0.75 mm** | **4.0 mm** | **0.187 / 0.574 mm** |
| 0.47 m | 0.75 mm | 6.0 mm | 0.187 / 0.573 mm |
| 0.47 m | 1.00 mm | 4.0 mm | 0.187 / 0.577 mm |
| 0.6 m | 0.50 mm | 2.5 mm | 0.393 / 1.075 mm |
| 0.6 m | 0.50 mm | 4.0 mm | 0.355 / 0.950 mm |
| 0.6 m | 0.75 mm | 2.5 mm | 0.366 / 1.027 mm |
| 0.6 m | **0.75 mm** | **4.0 mm** | **0.334 / 0.899 mm** |
| 0.6 m | 0.75 mm | 6.0 mm | 0.321 / 0.854 mm |
| 0.6 m | 1.00 mm | 4.0 mm | 0.326 / 0.878 mm |

With correlated noise the fusion parameters hardly matter: the error comes from the per-frame shape distortion
(waviness) and the registration error it causes (0.19° mean at 0.47 m, 0.29° at 0.6 m, against 0.04° without it).
At 0.47 m every setting is within 5 %; 0.5 mm voxels buy nothing for 2.4× the vertices. At 0.6 m a short 2.5 mm
truncation costs 10 %, and 6 mm gains 4 %. **The defaults (0.75 mm, 4 mm) stay.** Do not shorten the truncation
below 4 mm; with measured noise above ~1.3 mm at the working distance, raise it to 3σ. To be re-checked on real
recordings once the capture module's measurements (`docs/CAMERA.md`) are in.

**Memory.** A head volume with margins is about 0.27 × 0.33 × 0.30 m. That is about 60 M voxels (0.5 GB) at
0.75 mm and about 200 M voxels (1.6 GB) at 0.5 mm.

## Registration accuracy and capture range

**Whole-scan registration.** The test uses 11 rendered views of the synthetic head at 640×400: front ×3, up ×2,
down ×2, left ×2, right ×2. Depth noise is 0.9 mm at 0.6 m, and the initial poses are perturbed by 4° about the
head plus 6 mm. With the default (colored) ICP parameters of `fr_registration_params_default`, every frame lands
within **0.27 mm** of the truth (the worst displacement of head points), with rotation error ≤ 0.10°. Edge RMSE is
0.6 mm, which is the noise level.

**Capture range.** Same views, 3 random perturbations per row; time is per registration:

| initial rotation error | initial translation error | colored ICP | worst misalignment (mm) | max rejected edges | time (s) |
|---|---|---|---|---|---|
| 2° | 3 mm | yes | 0.30 | 0 | 0.07 |
| 2° | 3 mm | no | 0.72 | 0 | 0.06 |
| 4° | 6 mm | yes | 0.30 | 0 | 0.07 |
| 4° | 6 mm | no | 0.71 | 0 | 0.06 |
| 6° | 10 mm | yes | 0.28 | 0 | 0.07 |
| 6° | 10 mm | no | 0.68 | 0 | 0.06 |
| 8° | 15 mm | yes | 0.42 | 5 | 0.09 |
| 8° | 15 mm | no | 0.75 | 5 | 0.08 |
| 10° | 20 mm | yes | 0.44 | 16 | 0.09 |
| 10° | 20 mm | no | 0.84 | 16 | 0.09 |

(`IFS_BENCH=1 scripts/test.sh -c release --filter ReconCaptureRange`, with the robust photometric kernel; before it,
colored ICP reached 0.41–0.54 mm.) The perturbations are per frame, so relative errors between two frames are up
to twice these values. Colored ICP is about 50 % more accurate on this smooth test head, where geometry alone
leaves sliding directions and the texture looks the same from every view; on the textured head with view-dependent
shading it is not (next section).

### Colored versus point-to-plane ICP (default: point-to-plane)

The capture-range table above uses this module's own test head, whose color is identical from every view. The
textured head of `SyntheticData` (`ifscan synth`) also renders view-dependent shading from a light at the camera
(`--lighting`), which is what a real head turning under fixed room lights looks like. Full pipeline (`ifscan
reconstruct --icp colored|geometric`, initial poses from the dataset's simulated Vision poses and landmarks, TSDF
0.75 mm, 1280×800, noise 2 mm at 1 m, `--triangles 60000`), errors against the ground truth. "Before" is the
integrated version (f91b9c6), "after" has the photometric kernel and exposure gain (above) and the cloud fix for
isolated holes ("Point clouds"), which also helps point-to-plane:

| scan | lighting | ICP | version | rotation error mean / max | head misalignment mean / max | surface mean / p95 |
|---|---|---|---|---|---|---|
| 5 poses × 2 | 0.6 | colored | before | 2.68° / 8.38° | 6.8 / 19.3 mm | 1.59 / 4.47 mm |
| 5 poses × 2 | 0.6 | colored | after | 1.86° / 3.60° | 4.7 / 8.5 mm | 1.25 / 3.70 mm |
| 5 poses × 2 | 0.6 | point-to-plane | before | 0.06° / 0.12° | 0.14 / 0.25 mm | 0.13 / 0.39 mm |
| 5 poses × 2 | 0.6 | point-to-plane | after | **0.035° / 0.066°** | **0.09 / 0.16 mm** | **0.13 / 0.38 mm** |
| 5 poses × 2 | 0.25 | colored | after | 0.56° / 1.28° | 1.6 / 3.7 mm | 0.40 / 1.30 mm |
| 5 poses × 2 | 0.25 | point-to-plane | after | **0.035° / 0.066°** | **0.09 / 0.16 mm** | **0.13 / 0.38 mm** |
| 5 poses × 2 | 0 (flat) | colored | after | 0.068° / 0.110° | 0.17 / 0.33 mm | 0.13 / 0.40 mm |
| 5 poses × 2 | 0 (flat) | point-to-plane | after | **0.035° / 0.066°** | **0.09 / 0.16 mm** | **0.13 / 0.38 mm** |
| 9 poses × 2 | 0 (flat) | colored | before | 0.38° / 0.81° | 1.3 / 2.8 mm | 0.27 / 0.61 mm |
| 9 poses × 2 | 0 (flat) | colored | after | 0.049° / 0.084° | 0.14 / 0.25 mm | 0.08 / 0.22 mm |
| 9 poses × 2 | 0 (flat) | point-to-plane | before | 0.08° / 0.15° | 0.19 / 0.35 mm | 0.09 / 0.25 mm |
| 9 poses × 2 | 0 (flat) | point-to-plane | after | **0.047° / 0.083°** | **0.13 / 0.22 mm** | **0.08 / 0.23 mm** |

**Why colored ICP was worse: a defect and an inherent limit.** Measured pair by pair (`ifscan synth` views, ICP
started at the true relative pose, so any movement is bias):

* **Defect: no robust weighting of the photometric residual.** With flat lighting and noise-free depth, colored
  ICP drifted 0.26° (mean over 8 pairs) away from the truth, point-to-plane 0.008°. At the true pose the median
  |r_C| is 0.005–0.007, the color noise (1.5 of 255 levels). But 1.4–2.7 % of the correspondences have |r_C| > 0.1:
  texture edges (eyes, brows, lips, hairline, freckles), where the intensity gradient is ~5× the typical one (26–53
  versus 9–12 per meter) and the linear model I_p + d_p·δ fails over the ~0.5 mm offset δ to the nearest target
  point. With the quadratic cost they dominated the normal equations: one Gauss-Newton step from the truth moved
  0.08–0.62° with them and 0.03–0.17° without them. With the Tukey kernel the drift is 0.038° (noise-free) and
  0.053° versus 0.046° for point-to-plane on noisy flat-lit pairs (11 pairs). The additive offset c could not absorb
  auto exposure: with ±15 % gain per view the drift was 0.17° with c and is 0.052° with the gain g.
* **Inherent: view-dependent shading.** With lighting 0.6, half of the correspondences of a front/left pair have
  |r_C| > 0.1 at the true pose: the same skin point is brighter facing the light. Brightness constancy, the model of
  colored ICP, does not hold; the term aligns the shading, which moves with the light rather than with the head.
  The kernel cannot separate this from texture. Locally normalized log intensity (an experiment, not adopted)
  brought lighting-0.6 pairs from 2.0° to 0.21°, but made flat-lit pairs worse (0.18°).
* **Checked and not the cause:** pixel-center convention (the renderer, `fr_cloud_from_rgbd` and the TSDF projection
  all use integer pixel centers, (x − cx) / fx, and color and depth come from the same pixel), the tangent-plane
  projection of d_p, and the gradient neighborhood (normal radius 1, 2, 3 or 5 pixels: 0.024–0.050° with the
  kernel).

**Decision unchanged: point-to-plane is the default.** Even fixed, colored ICP only matches point-to-plane with
perfectly flat lighting and loses to it under any shading. A head is not a sliding-prone surface (nose, eye
sockets, ears), so point-to-plane is well constrained. A real D455 adds more reasons: specular skin highlights move
with the view, and color and aligned depth disagree by up to about a pixel (factory calibration).
`ReconstructionOptions.useColoredICP` stays `false`; colored ICP remains
available (`--icp colored`) for smooth, strongly textured surfaces under diffuse light. To be re-checked on real
recordings.

**40-keyframe scan.** In the 1280×800 benchmark scan, the worst frame is 0.87 mm off (a down-left frame) and the
median is about 0.36 mm; all 40 keyframes pass the registration check.

## Robustness to real-data imperfections

The synthetic renderer has independent depth noise, depth wherever the surface is visible, exact intensities and
the scan poses in order. `PipelineRobustnessTests` adds what real D455 scans have, one at a time and all together
(textured `SyntheticData` head, 5 poses × 2 at 1280×800 and 0.6 m, lighting 0.25, full pipeline; `IFS_RECON_STUDY=1`
prints the table, the default run checks the combined scenario):

| scenario | ICP | rotation error mean / max | head misalignment max | surface mean / p95 |
|---|---|---|---|---|
| clean | point-to-plane | 0.04° / 0.07° | 0.16 mm | 0.12 / 0.38 mm |
| clean | colored | 0.56° / 1.28° | 3.74 mm | 0.40 / 1.30 mm |
| correlated noise 0.5 mm + waviness 0.3 mm (at 0.6 m) | point-to-plane | 0.29° / 0.52° | 1.30 mm | 0.33 / 0.90 mm |
| correlated noise 0.5 mm + waviness 0.3 mm (at 0.6 m) | colored | 0.73° / 1.48° | 3.81 mm | 0.54 / 1.65 mm |
| no depth on 80 % of dark pixels (hair, brows, pupils) | point-to-plane | 0.04° / 0.07° | 0.18 mm | 0.14 / 0.42 mm |
| no depth on 80 % of dark pixels (hair, brows, pupils) | colored | 0.65° / 1.67° | 4.03 mm | 0.39 / 1.63 mm |
| silhouettes eroded by 6 px | point-to-plane | 0.05° / 0.11° | 0.28 mm | 0.14 / 0.43 mm |
| silhouettes eroded by 6 px | colored | 0.66° / 1.34° | 2.96 mm | 0.33 / 1.24 mm |
| Vision error 4° per angle (σ) | point-to-plane | 0.04° / 0.07° | 0.16 mm | 0.12 / 0.38 mm |
| Vision error 4° per angle (σ) | colored | 0.57° / 1.37° | 4.07 mm | 0.47 / 1.64 mm |
| exposure ±20 % per keyframe | point-to-plane | 0.04° / 0.07° | 0.16 mm | 0.12 / 0.38 mm |
| exposure ±20 % per keyframe | colored | 0.54° / 1.15° | 3.29 mm | 0.33 / 1.04 mm |
| frontal keyframes last in the capture | point-to-plane | 0.04° / 0.07° | 0.16 mm | 0.13 / 0.38 mm |
| frontal keyframes last in the capture | colored | 0.56° / 1.28° | 3.74 mm | 0.40 / 1.29 mm |
| 8 % scattered depth holes | point-to-plane | 0.04° / 0.06° | 0.18 mm | 0.14 / 0.41 mm |
| 8 % scattered depth holes | colored | 0.56° / 1.24° | 3.58 mm | 0.39 / 1.27 mm |
| all combined (Vision 3°, 5 % holes) | point-to-plane | 0.40° / 0.87° | 2.57 mm | 0.36 / 0.97 mm |
| all combined (Vision 3°, 5 % holes) | colored | 1.05° / 1.95° | 4.78 mm | 0.71 / 2.40 mm |

* Correlated noise and low-frequency waviness are the one imperfection that costs accuracy, and it is inherent:
  each frame's surface is slightly deformed, so no rigid registration fits all views exactly.
* Scattered depth holes failed the whole reconstruction before the cloud fix ("Point clouds"): with 8 % holes no
  keyframe but the reference passed the registration check (`notEnoughKeyframes(1)`).
* Vision errors up to σ = 4° per angle do not matter: landmark Kabsch fits replace the Vision rotation (all 9
  non-reference frames of the combined scenario) and ICP converges from there.
* The capture order does not matter: the pipeline sorts by scan pose, and the reference stays the first frontal
  keyframe.
* Missing on purpose: specular highlights, rolling shutter and motion blur during the hold, and a depth bias that
  depends on the surface material (skin subsurface scattering). Real recordings are the only check for those.

## Mesh cleanup (`MeshProcessing.clean`)

The mesh module's steps after the chin crop, all in FaceReconCore (`fr_mesh.h`):

1. **Weld** vertices within 1 µm (`fr_mesh_clean`), then drop degenerate and duplicate triangles, and on every edge
   that is still used by more than two triangles or twice in one direction all but the largest triangle per
   direction. Marching-cubes vertices near a voxel corner lie within a micrometer of each other (318 such pairs on
   the synthetic head below), and welding them collapses edges without regard to the topology: four vertices merged
   into one left an edge with three triangles (one a 10⁻⁴ mm² sliver) — the non-manifold edge every export of that
   scan carried. Hole filling closes the gap the removed triangle leaves.
2. **Small components** (below 5 % of the largest; connected through edges).
3. **Smoothing**: bilateral normal filtering, then Taubin (`fr_mesh_smooth`).
4. **Hole filling and closing** (`fr_mesh_fill_holes_ex`, below).
5. **Decimation** to `targetTriangles` if set (`fr_mesh_decimate`, meshoptimizer). The simplifier does not keep the
   surface manifold (a closed 556k-triangle head decimated to 60k had 74 edges with three or four triangles); those
   extra triangles go and the small holes they leave are filled again.

`MeshProcessing.clean` returns a summary (holes filled, loops left open, closed or not, trimmed and folded
triangles, filled area) and sets `ReconMesh.synthesized` for the vertices the filling created.

### Holes

Every boundary loop but the longest one is a hole. A hole whose border is at most `maxHolePerimeterMM` long
(250 mm; `maxHoleEdges`, 4096, is only a safety cap) is triangulated with Liepa's minimal dihedral-angle / area
dynamic program, refined to the density of the surrounding mesh and faired as a thin plate. The limit used to be 400
edges, which left the openings behind both ears open: on the synthetic head below they are 166 mm / 430 edges and
162 mm / 420 edges at 0.75 mm voxels (the border of a marching-cubes surface is ragged: ~0.4 mm per edge), and more
edges at finer voxels.

A patch is checked against the surrounding surface: a triangle is folded when its normal points against the summed
area-weighted normals of its corners (the test of the smoothing's fold guard), counted for the patch and for the
measured triangles at its border. A folded patch is flipped to Delaunay and faired again, then smoothed locally, then
replaced by the unrefined triangulation, and otherwise discarded. On the synthetic head the filling took the mesh from
14 folded triangles after smoothing to 32 (580 holes); now it adds none, and 10 holes whose every patch folds stay
open (one is the 162 mm opening behind the left ear, a deep cup whose best patch folds 82 of 418 triangles) — unless
the head is closed, which asks for a watertight result: then every loop is filled with its least folded patch.

### Closing the head (`ReconstructionOptions.closeHead`, default on)

The guided scan never sees the back of the head (|yaw| ≤ 45°), so the longest loop is the open back joined with the
neck cut: 1777 mm and 4661 edges on the synthetic head. Left open, the model is invisible from behind in one-sided
viewers (QuickLook, Preview cull back faces) and shows its inside from the side. `closeHead` closes it into a
watertight bust:

1. **Fringe.** The low-weight fringe of the fusion along the loop (teeth with two border edges, triangles folded over
   by more than 75° against a neighbor) is trimmed, pieces that cuts off go, and the notches of the zigzag border get
   triangles of loop vertices only.
2. **Center and cut.** A sphere fitted to the measured band along the loop (a tenth of its extent wide, without the
   crop) gives the head center; the planar run of the loop gives the chin crop's plane (crop vertices lie on it to
   float precision: 597 consecutive loop vertices within 1.3 µm).
3. **Triangulation.** The loop's directions from the center, stereographically projected from the front, form a
   nearly simple polygon (local zigzags are straightened); ear clipping, then Liepa refinement graded from the loop's
   edge length up to 4× the mean edge length (2.6 mm here) and Delaunay flips, all in that plane, where they cannot
   fold.
4. **Shape.** The new vertices keep their directions; their distance from the center is a thin plate of the loop's
   distances (cotangent weights on the second solve), bounded within ±20 % around the membrane solution, and capped
   where the ray leaves through the crop plane (an active set: held vertices are boundary values). A head is
   star-shaped around its center and its distance function smooth, so the closing continues its roundness — a
   Cartesian thin plate flattens such an opening (on a 100 mm sphere opened above y = 20 mm it stayed 10.9 mm off
   the sphere on average, 26 mm at most; in spherical coordinates the sphere comes back exactly). The loop vertices
   contribute their distance, not its slope: matching the steep slope at an ear's rim overshot into spikes. Where
   rays leave through the crop the closing is a flat cap: the neck becomes the flat bottom of a bust.
5. **Repair.** Remaining folds are smoothed locally. Measured triangles the closing still folds, and those across the
   loop edges of folded closing triangles, are trimmed and the closing is done again (at most five attempts, ~0.7 s
   each; the filled holes stay): on the two synthetic heads 48 and 141 folded triangles after three attempts, 37 and
   110 after five.

Without a usable center (the band does not face away from it) the closing falls back to a refined fan faired in
Cartesian coordinates. Every other loop is filled regardless of the perimeter limit, so the result is watertight;
loops beyond it (other openings of the same kind: 462 mm and 316 mm on the head of the 40-keyframe benchmark) are
trimmed and closed like the longest one.

Measured on the synthetic dataset of `ifscan synth --lighting 0.6` (10 keyframes, 1280×800, 0.75 mm voxels), release
build, exported OBJ welded by position as viewers do:

| | before | `closeHead` off | `closeHead` on |
|---|---|---|---|
| boundary loops | 4 (1777, 166, 162 mm; a 2-edge pinch) | 11 (1777 mm open; 10 holes left by the fold check) | 0: every edge in exactly two triangles, consistently oriented, outward |
| non-manifold edges | 1 | 0 | 0 |
| folded triangles | 32 | 13–14 | 28–53 (varies between runs: the Metal fusion is not bit-exact) |
| intersecting triangle pairs | 145 | 138 | 143–162 |
| synthesized area | – | 15 cm² of 857 cm² | 549 cm² of 1390 cm² (trimmed fringe: 1326 triangles) |
| surface to ground truth, measured part: mean / p95 / max | 0.129 / 0.390 / 2.36 mm | 0.128 / 0.387 / 2.54 mm | 0.128 / 0.389 / 2.31 mm |
| surface to ground truth, whole mesh: mean / p95 / max | 0.129 / 0.390 / 2.36 mm | 0.130 / 0.393 / 2.36 mm | 3.54 / 16.9 / 54.5 mm |
| cleanup | 0.4 s | 1.2 s | 4.3–5.4 s |

Intersecting pairs are triangles that share no vertex and cross each other: tiny ones along ragged borders and in the
ears; the measured surface has about a hundred of its own. The measured surface is unchanged. Measured against the
ground truth as a whole, the closed mesh is far off, by design: the synthesized surface is a plausible guess of what
no keyframe saw. The flat cap lies across the true neck (which goes on down), 25–55 mm from its wall, and the back of
the neck is fuller than the truth. Accuracy figures belong to `ReconMesh.measuredSurface`;
`ReconstructionReport.synthesizedAreaMM2` (and a note) says how much was made up.

SceneKit renders of the export (QuickLook's single-sided materials) from the front, both sides, behind, below and
above show a closed head: a round back continuing the skull, the openings behind the ears closed, a flat neck. Small
pinches remain where the cap meets the dome at the back corners of the neck.

## Texturing (`Texturing`)

`colorizeVertices` and `unwrapAndBake` blend the best views per surface point (view angle, quality weight,
feathering at silhouettes, depth test, exposure compensation). The synthesized vertices (`ReconMesh.synthesized`,
passed as `fr_texture_params.synthesized`) are never colored from the keyframes — the frames would project whatever
they saw along those rays onto a guessed surface — and are left out of the exposure compensation. They, and measured
vertices no frame sees, take a harmonic interpolation (uniform graph Laplacian) of the surrounding seen colors, whose
border values are blurred over three rings first: on a head the back fades smoothly from the hair at the crown to the
skin of the neck, instead of the radial streaks of the earlier wave-by-wave diffusion or one global mean color. The
texture fades the frames out toward synthesized corners and uses the interpolated colors there. The flags follow the
vertices through UV seam splits and decimation (`withCoreMesh`, by position).

UV unwrapping (xatlas, faces split into chart groups of 10 000) dominates the texturing time: 13–18 s for the
550k-triangle closed head above, 11.4 s in the 40-keyframe benchmark (10.5 s at integration, before the closing added
40 % of surface).

## Export (`MeshExport`)

OBJ (+MTL +PNG) and PLY in millimeters in the export frame (+Y up, face toward +Z, origin at the head center). A
textured OBJ carries its color in the texture only: viewers multiply the texture by "v x y z r g b" colors, so
`ExportOptions.includeVertexColors` applies to untextured OBJs and to the PLY. UV seams split vertices in the file;
welded by position (as viewers do), the closed export is watertight.

## Timings

Measured on an Apple M3 Ultra (28 CPU cores, 96 GB) in a release build: the full pipeline on 40 keyframes at
1280×800 with 0.75 mm voxels on Metal (`PipelineBenchmark.fortyKeyframesAt1280x800`, after integration). The task's
reference machine is an M4 Pro; this one is somewhat faster on GPU and CPU throughput.

| stage | seconds |
|---|---|
| segmentation (40 frames, concurrent) | 0.03 |
| registration (clouds, 172 ICP edges, pose graph) | 0.8–1.5 |
| registration check (pairs across pose groups) | 0.59 |
| bounds | 0.24–0.38 |
| fusion (TSDF, Metal; CPU path 0.4 s) and marching cubes | 0.17 |
| cleanup (crop + `MeshProcessing.clean`) | 0.25 (closing the head: 6.1) |
| texturing (mesh module, full-resolution mesh) | 10.5 (closed head, 40 % more surface: 11.4) |
| **total** | **13.5** (closed head: 20.2) |

This is inside the 60 s target; texturing dominates. The figures in parentheses are the same benchmark with
`closeHead` on (the default since then), measured separately. The registration check was 0.15 s when it compared each frame
with one view only; it now evaluates about 6 pairs per frame. The debug-build unit tests run the 13-keyframe
640×400 pipeline in 1–3 s plus texturing.

## Known limits

* **The back of the head is not observed** by the guided-scan poses (|yaw| ≤ 45°). `closeHead` (default) closes it
  with a synthesized surface ("Closing the head"): plausible, not measured — 40 % of the closed head's surface, a
  flat cap at the chin crop, and accuracy figures only for `ReconMesh.measuredSurface`. Tests that compare a whole
  closed mesh with a ground truth measure the guess (3.5 mm mean on the synthetic head, 25–55 mm on the cap inside
  the true neck). A measured back needs back or over-the-shoulder views from the scan module.
* **The closing is star-shaped** around the fitted head center: where the measured border is not (an ear's rim on
  the loop), the closing meets it with a crease rather than following it; small pinches remain where the flat cap
  meets the dome at the back corners of the neck.
* **Decimation folds triangles.** meshoptimizer's simplifier keeps the surface manifold after the repair, but on the
  closed head decimated to 60k triangles about 480 triangles end up folded by the fold test (about 510 on the open
  head before the closing), against about 30–50 at full resolution.
* **Frames that cannot be registered** keep their initial pose inside `fr_register_frames`. The C API has no
  per-frame status, so the pipeline re-scores the registered clouds (`RegistrationCheck`, below) and leaves failing
  frames out of fusion.
* **Registration check blind spot.** An error that rotates a frame about the head center mostly slides the
  near-spherical skull along itself; only the face features show it, and the surface check does not see a few
  degrees of it. The pose graph and the edge rejection are the defense there.
* **Export orientation.** Yaw and pitch come from the mean of all keyframes' Vision head poses carried into the
  world (outliers more than 10° off ignored). Roll comes from the reference eye line. A systematic Vision yaw bias
  for a person would tilt the export frame by that bias. A symmetry-plane fit on the mesh would remove it.
* **Chin crop fallback.** `fr_mesh_crop_plane` cuts the triangles at the plane; only if it fails does the pipeline
  drop whole triangles below the plane (jagged border, and no planar run for the closing's flat cap) and note it.
* **Vertex colors from the TSDF** are count-averaged in RGBA8. They are a fallback: the texture and vertex colors
  of the mesh module replace them.
