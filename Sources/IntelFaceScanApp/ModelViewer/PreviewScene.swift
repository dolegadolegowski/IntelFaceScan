// License: Apache 2.0. See LICENSE file in root directory.
//
// SceneKit scene of a reconstruction result for the 3D preview: geometry sources are built once (in millimeters, in
// the export frame: +Y up, face toward +Z); display modes only swap geometry/material pairs that share them.

import AppKit
import ReconKit
import SceneKit
import simd

/// Cheap identity of a result, to rebuild the scene only when the model actually changed.
struct PreviewFingerprint: Equatable {
    var vertexCount: Int
    var triangleCount: Int
    var textureSize: SIMD2<Int>
    var samples: [SIMD3<Float>]
    var transform: [SIMD4<Float>]

    init(_ result: ReconstructionResult) {
        let mesh = result.mesh
        vertexCount = mesh.vertexCount
        triangleCount = mesh.triangleCount
        textureSize = SIMD2(result.texture?.width ?? 0, result.texture?.height ?? 0)
        let n = mesh.positions.count
        samples = n == 0 ? [] : [0, n / 3, n / 2, (2 * n) / 3, n - 1].map { mesh.positions[$0] }
        let m = result.exportTransform
        transform = [m.columns.0, m.columns.1, m.columns.2, m.columns.3]
    }
}

final class PreviewScene {
    /// What a result can show besides shading: a texture needs one UV per vertex, colors one color per vertex.
    struct Content: Equatable {
        var hasTexture: Bool
        var hasColors: Bool

        init(hasTexture: Bool, hasColors: Bool) {
            self.hasTexture = hasTexture
            self.hasColors = hasColors
        }

        init(_ result: ReconstructionResult) {
            let mesh = result.mesh
            let n = mesh.vertexCount
            hasTexture = result.texture != nil && mesh.uvs.count == n && n > 0
            hasColors = mesh.colors.count == n && n > 0
        }

        /// The mode actually shown for a request (texture and colors fall back when the result has none).
        func effectiveMode(_ mode: ModelDisplayMode) -> ModelDisplayMode {
            switch mode {
            case .texture: return hasTexture ? .texture : (hasColors ? .vertexColors : .shaded)
            case .vertexColors: return hasColors ? .vertexColors : .shaded
            case .shaded, .wireframe: return mode
            }
        }
    }

    let scene = SCNScene()
    let cameraNode = SCNNode()
    let content: Content
    var hasTexture: Bool { content.hasTexture }
    var hasColors: Bool { content.hasColors }
    /// Bounds of the model in millimeters (export frame).
    let boundsMin: SIMD3<Float>
    let boundsMax: SIMD3<Float>
    let triangleCount: Int

    private let modelNode = SCNNode()
    private let wireNode = SCNNode()
    private let plainGeometry: SCNGeometry
    private let colorGeometry: SCNGeometry?
    private var materials: [ModelDisplayMode: SCNMaterial] = [:]

    init(result: ReconstructionResult) {
        let mesh = result.mesh
        let n = mesh.vertexCount
        triangleCount = mesh.triangleCount
        let transform = result.exportTransform
        let rotation = simd_float3x3(SIMD3(transform.columns.0.x, transform.columns.0.y, transform.columns.0.z),
                                     SIMD3(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z),
                                     SIMD3(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z))
        let normalMatrix = rotation.inverse.transpose
        // Valid triangles only: a malformed index must not take the viewer down.
        var indices = [UInt32]()
        indices.reserveCapacity(mesh.indices.count)
        for t in 0 ..< mesh.triangleCount {
            let a = mesh.indices[3 * t], b = mesh.indices[3 * t + 1], c = mesh.indices[3 * t + 2]
            if Int(a) < n, Int(b) < n, Int(c) < n { indices += [a, b, c] }
        }

        var positions = [Float](repeating: 0, count: 3 * n)
        var lo = SIMD3<Float>(repeating: .greatestFiniteMagnitude), hi = -lo
        for i in 0 ..< n {
            let p4 = transform * SIMD4(mesh.positions[i], 1)
            let p = SIMD3(p4.x, p4.y, p4.z) * 1000
            positions[3 * i] = p.x; positions[3 * i + 1] = p.y; positions[3 * i + 2] = p.z
            if p.x.isFinite, p.y.isFinite, p.z.isFinite { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        }
        if n == 0 || lo.x > hi.x { lo = .zero; hi = .zero }
        boundsMin = lo
        boundsMax = hi

        var normals = [Float](repeating: 0, count: 3 * n)
        if mesh.normals.count == n {
            for i in 0 ..< n {
                let v = simd_normalize(normalMatrix * mesh.normals[i])
                normals[3 * i] = v.x; normals[3 * i + 1] = v.y; normals[3 * i + 2] = v.z
            }
        } else {
            Self.computeNormals(positions: positions, indices: indices, into: &normals)
        }

        let positionSource = SCNGeometrySource(data: Self.data(positions), semantic: .vertex, vectorCount: n,
                                               usesFloatComponents: true, componentsPerVector: 3, bytesPerComponent: 4,
                                               dataOffset: 0, dataStride: 12)
        let normalSource = SCNGeometrySource(data: Self.data(normals), semantic: .normal, vectorCount: n,
                                             usesFloatComponents: true, componentsPerVector: 3, bytesPerComponent: 4,
                                             dataOffset: 0, dataStride: 12)
        let element = SCNGeometryElement(data: Self.data(indices), primitiveType: .triangles,
                                         primitiveCount: indices.count / 3, bytesPerIndex: 4)

        var plainSources = [positionSource, normalSource]
        content = Content(result)
        if content.hasTexture {
            // SceneKit texture coordinates have their origin at the top-left of the image, OBJ UVs at the bottom-left.
            var uvs = [Float](repeating: 0, count: 2 * n)
            for i in 0 ..< n { uvs[2 * i] = mesh.uvs[i].x; uvs[2 * i + 1] = 1 - mesh.uvs[i].y }
            plainSources.append(SCNGeometrySource(data: Self.data(uvs), semantic: .texcoord, vectorCount: n,
                                                  usesFloatComponents: true, componentsPerVector: 2, bytesPerComponent: 4,
                                                  dataOffset: 0, dataStride: 8))
        }
        plainGeometry = SCNGeometry(sources: plainSources, elements: [element])

        if content.hasColors {
            // Float components: SceneKit does not normalize 8-bit vertex colors.
            var rgba = [Float](repeating: 1, count: 4 * n)
            for i in 0 ..< n {
                rgba[4 * i] = Float(mesh.colors[i].x) / 255
                rgba[4 * i + 1] = Float(mesh.colors[i].y) / 255
                rgba[4 * i + 2] = Float(mesh.colors[i].z) / 255
            }
            let colorSource = SCNGeometrySource(data: Self.data(rgba), semantic: .color, vectorCount: n,
                                                usesFloatComponents: true, componentsPerVector: 4, bytesPerComponent: 4,
                                                dataOffset: 0, dataStride: 16)
            colorGeometry = SCNGeometry(sources: [positionSource, normalSource, colorSource], elements: [element])
        } else {
            colorGeometry = nil
        }

        materials[.texture] = Self.textureMaterial(result.texture)
        materials[.vertexColors] = Self.surfaceMaterial(color: .white, lighting: .lambert)
        materials[.shaded] = Self.surfaceMaterial(color: NSColor(calibratedRed: 0.80, green: 0.77, blue: 0.74, alpha: 1),
                                                  lighting: .blinn)
        materials[.wireframe] = Self.surfaceMaterial(color: NSColor(calibratedRed: 0.20, green: 0.23, blue: 0.27, alpha: 1),
                                                     lighting: .lambert)

        // Wireframe = dark surface plus its edges on a copy of the geometry (shared sources), pushed outward by a
        // tenth of a millimeter so the lines win the depth test; dense meshes still show their shape.
        let lines = plainGeometry.copy() as? SCNGeometry ?? SCNGeometry(sources: plainSources, elements: [element])
        let wire = Self.wireframeMaterial()
        // Fainter lines on dense meshes, which otherwise read as a solid fill at the default zoom.
        wire.transparency = mesh.triangleCount < 150_000 ? 0.55 : (mesh.triangleCount < 600_000 ? 0.35 : 0.22)
        lines.materials = [wire]
        wireNode.geometry = lines
        let scale = 1 + 0.1 / max(1, simd_length(hi - lo) / 2)
        let c = (lo + hi) / 2
        wireNode.simdTransform = simd_float4x4(translation: c) * simd_float4x4(diagonal: SIMD4(scale, scale, scale, 1))
            * simd_float4x4(translation: -c)
        wireNode.isHidden = true
        wireNode.renderingOrder = 1

        modelNode.geometry = plainGeometry
        modelNode.addChildNode(wireNode)
        scene.rootNode.addChildNode(modelNode)
        setUpCameraAndLights()
    }

    func effectiveMode(_ mode: ModelDisplayMode) -> ModelDisplayMode { content.effectiveMode(mode) }

    func apply(_ requested: ModelDisplayMode) {
        let mode = effectiveMode(requested)
        let geometry = mode == .vertexColors ? (colorGeometry ?? plainGeometry) : plainGeometry
        if let material = materials[mode] { geometry.materials = [material] }
        SCNTransaction.begin()
        SCNTransaction.disableActions = true
        modelNode.geometry = geometry
        wireNode.isHidden = mode != .wireframe
        SCNTransaction.commit()
    }

    var center: SIMD3<Float> { (boundsMin + boundsMax) / 2 }
    var radius: Float { max(1, simd_length(boundsMax - boundsMin) / 2) }

    /// Looks at the face (from +Z) with the whole model in view.
    func frameCamera(in view: SCNView?) {
        guard let camera = cameraNode.camera else { return }
        let fov = Float(camera.fieldOfView) * .pi / 180
        let distance = radius / sin(fov / 2) * 1.05
        let c = center
        cameraNode.simdPosition = SIMD3(c.x, c.y, c.z + distance)
        cameraNode.simdLook(at: c, up: SIMD3(0, 1, 0), localFront: SIMD3(0, 0, -1))
        camera.zNear = Double(max(0.5, distance - 4 * radius))
        camera.zFar = Double(distance + 4 * radius)
        if let view {
            view.pointOfView = cameraNode
            view.defaultCameraController.target = SCNVector3(c.x, c.y, c.z)
            view.defaultCameraController.worldUp = SCNVector3(0, 1, 0)
        }
    }

    // MARK: - Scene setup

    private func setUpCameraAndLights() {
        let camera = SCNCamera()
        camera.fieldOfView = 30
        camera.automaticallyAdjustsZRange = true
        cameraNode.camera = camera
        cameraNode.name = "preview camera"
        scene.rootNode.addChildNode(cameraNode)

        // Lights ride with the camera so the shading reads the same from every side; a soft ambient term keeps the
        // texture colors visible in the shadows.
        let ambient = SCNLight()
        ambient.type = .ambient
        ambient.intensity = 280
        ambient.color = NSColor(white: 1, alpha: 1)
        let ambientNode = SCNNode()
        ambientNode.light = ambient
        scene.rootNode.addChildNode(ambientNode)
        func directional(_ intensity: CGFloat, yaw: Float, pitch: Float) -> SCNNode {
            let light = SCNLight()
            light.type = .directional
            light.intensity = intensity
            let node = SCNNode()
            node.light = light
            node.simdEulerAngles = SIMD3(pitch, yaw, 0)
            return node
        }
        // A light shines along its node's -Z: a negative yaw turns it toward +X, i.e. it comes from the left.
        cameraNode.addChildNode(directional(780, yaw: -0.45, pitch: -0.5))   // key: upper left
        cameraNode.addChildNode(directional(260, yaw: 0.7, pitch: 0.1))      // fill: right
        cameraNode.addChildNode(directional(160, yaw: 3.0, pitch: -0.3))     // rim: behind
        frameCamera(in: nil)
    }

    private static func textureMaterial(_ texture: TextureImage?) -> SCNMaterial {
        let material = surfaceMaterial(color: .white, lighting: .lambert)
        if let texture, let image = cgImage(texture) {
            material.diffuse.contents = image
            material.diffuse.mipFilter = .linear
            material.diffuse.minificationFilter = .linear
            material.diffuse.magnificationFilter = .linear
            material.diffuse.wrapS = .clamp
            material.diffuse.wrapT = .clamp
        }
        return material
    }

    private static func surfaceMaterial(color: NSColor, lighting: SCNMaterial.LightingModel) -> SCNMaterial {
        let material = SCNMaterial()
        material.lightingModel = lighting
        material.diffuse.contents = color
        material.specular.contents = NSColor(white: lighting == .blinn ? 0.18 : 0, alpha: 1)
        material.shininess = 0.2
        // The neck opening shows the inside of the surface.
        material.isDoubleSided = true
        material.locksAmbientWithDiffuse = true
        return material
    }

    private static func wireframeMaterial() -> SCNMaterial {
        let material = SCNMaterial()
        material.lightingModel = .constant
        material.fillMode = .lines
        material.diffuse.contents = NSColor(calibratedRed: 0.55, green: 0.80, blue: 1.0, alpha: 1)
        material.transparency = 0.55
        material.blendMode = .alpha
        material.writesToDepthBuffer = false
        material.isDoubleSided = false
        return material
    }

    private static func cgImage(_ texture: TextureImage) -> CGImage? {
        let w = texture.width, h = texture.height
        guard w > 0, h > 0, texture.rgba.count >= w * h * 4,
              let provider = CGDataProvider(data: texture.rgba as CFData),
              let space = CGColorSpace(name: CGColorSpace.sRGB) else { return nil }
        return CGImage(width: w, height: h, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: w * 4, space: space,
                       bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue), provider: provider,
                       decode: nil, shouldInterpolate: true, intent: .defaultIntent)
    }

    static func data<T>(_ array: [T]) -> Data {
        array.withUnsafeBufferPointer { Data(buffer: $0) }
    }

    private static func computeNormals(positions: [Float], indices: [UInt32], into normals: inout [Float]) {
        func p(_ i: UInt32) -> SIMD3<Float> {
            SIMD3(positions[3 * Int(i)], positions[3 * Int(i) + 1], positions[3 * Int(i) + 2])
        }
        var acc = [SIMD3<Float>](repeating: .zero, count: positions.count / 3)
        for t in stride(from: 0, to: indices.count, by: 3) {
            let a = indices[t], b = indices[t + 1], c = indices[t + 2]
            let fn = simd_cross(p(b) - p(a), p(c) - p(a))
            acc[Int(a)] += fn; acc[Int(b)] += fn; acc[Int(c)] += fn
        }
        for i in acc.indices {
            let l = simd_length(acc[i])
            let v = l > 0 ? acc[i] / l : SIMD3(0, 0, 1)
            normals[3 * i] = v.x; normals[3 * i + 1] = v.y; normals[3 * i + 2] = v.z
        }
    }
}

private extension simd_float4x4 {
    init(translation t: SIMD3<Float>) {
        self = matrix_identity_float4x4
        columns.3 = SIMD4(t, 1)
    }
}
