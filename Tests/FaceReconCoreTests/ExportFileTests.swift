// License: Apache 2.0. See LICENSE file in root directory.
//
// OBJ/MTL/PNG and PLY export: round trips through fr_import_obj and through ModelIO (the importer QuickLook and
// Preview use), PLY layout, OBJ import of foreign files, PNG writing.

import FaceReconCore
import Foundation
import ImageIO
import ModelIO
import ReconKit
import simd
import Testing

@Suite("Export")
struct ExportFileTests {
    /// Unwrapped sphere with normals, colors and a small baked texture.
    static func texturedSphere() throws -> (ReconMesh, TextureImage) {
        var mesh = try TextureBakingTests.unwrappedSphere(level: 3, size: 256)
        mesh.colors = mesh.positions.map { SIMD3<UInt8>(TexturedSphere.color(simd_normalize($0)).rounded(.toNearestOrEven)) }
        let views = TexturedSphere.surroundingPoses().map { SyntheticView.render(pose: $0, width: 160, height: 120, focal: 140) }
        let texture = try TextureBakingTests.bake(mesh, views: views, size: 256)
        return (mesh, texture)
    }

    static let rotation = simd_quatf(angle: .pi, axis: [1, 0, 0])  // camera (+Y down, +Z forward) -> upright
    static var transform: simd_float4x4 {
        var m = simd_float4x4(rotation)
        m.columns.3 = [0.01, 0.02, -0.03, 1]
        return m
    }

    static func exportOptions(transform: simd_float4x4 = transform) -> fr_export_options {
        var options = fr_export_options()
        fr_export_options_default(&options)
        let floats = transform.columnMajor
        withUnsafeMutableBytes(of: &options.transform) { raw in floats.withUnsafeBytes { raw.copyMemory(from: $0) } }
        options.has_transform = 1
        return options
    }

    static func exported(_ p: SIMD3<Float>, _ m: simd_float4x4 = transform) -> SIMD3<Float> {
        let q = m * SIMD4(p, 1)
        return SIMD3(q.x, q.y, q.z) * 1000
    }

    /// PNG color type from the IHDR chunk (0 gray, 2 RGB, 6 RGBA).
    static func pngColorType(_ url: URL) -> UInt8? {
        guard let data = try? Data(contentsOf: url), data.count > 25, data.prefix(8) == Data([137, 80, 78, 71, 13, 10, 26, 10]) else {
            return nil
        }
        return data[25]
    }

    @Test func objRoundTripInMillimeters() throws {
        let (mesh, texture) = try Self.texturedSphere()
        let dir = try TemporaryDirectory()
        let url = dir.url.appendingPathComponent("head scan.obj")
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        let coreTexture = try texture.makeCoreTexture()
        defer { fr_texture_free(coreTexture) }
        var options = Self.exportOptions()
        try "head".withCString { name in
            options.material_name = name
            try check(fr_export_obj(core, coreTexture, url.path, &options))
        }
        // Material and texture names are file-name safe and relative.
        let mtl = dir.url.appendingPathComponent("head_scan.mtl"), png = dir.url.appendingPathComponent("head_scan.png")
        #expect(FileManager.default.fileExists(atPath: mtl.path) && FileManager.default.fileExists(atPath: png.path))
        let objText = try String(contentsOf: url, encoding: .utf8)
        #expect(objText.contains("# units: millimeters"))
        #expect(objText.contains("mtllib head_scan.mtl\n") && objText.contains("usemtl head\n"))
        let mtlText = try String(contentsOf: mtl, encoding: .utf8)
        #expect(mtlText.contains("newmtl head\n") && mtlText.contains("map_Kd head_scan.png\n"))
        #expect(try FileManager.default.contentsOfDirectory(atPath: dir.url.path).filter { $0.hasSuffix(".partial") }.isEmpty)

        var imported: UnsafeMutablePointer<fr_mesh>?
        try check(fr_import_obj(url.path, &imported))
        let back = ReconMesh(core: try #require(imported))
        fr_mesh_free(imported)
        #expect(back.vertexCount == mesh.vertexCount && back.indices == mesh.indices)
        for i in 0 ..< mesh.vertexCount {
            #expect(simd_distance(back.positions[i], Self.exported(mesh.positions[i])) < 1e-3)  // mm
            #expect(simd_distance(back.normals[i], Self.rotation.act(mesh.normals[i])) < 2e-6)
            #expect(simd_distance(back.uvs[i], mesh.uvs[i]) < 1e-6)
            #expect(back.colors[i] == mesh.colors[i])
        }
        // Still counter-clockwise from outside after the 180 degree rotation.
        let center = Self.exported(.zero)
        let centered = ReconMesh(positions: back.positions.map { $0 - center }, indices: back.indices)
        #expect(MeshMeasure.signedVolume(centered) > 0)

        // The PNG holds the texture (RGB, same pixels).
        let source = try #require(CGImageSourceCreateWithURL(png as CFURL, nil))
        let image = try #require(CGImageSourceCreateImageAtIndex(source, 0, nil))
        #expect(image.width == texture.width && image.height == texture.height)
        #expect(Self.pngColorType(png) == 2)  // RGB, no alpha
    }

    @Test func modelIOLoadsTheExport() throws {
        let (mesh, texture) = try Self.texturedSphere()
        let dir = try TemporaryDirectory()
        let url = dir.url.appendingPathComponent("head.obj")
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        let coreTexture = try texture.makeCoreTexture()
        defer { fr_texture_free(coreTexture) }
        var options = Self.exportOptions()
        try check(fr_export_obj(core, coreTexture, url.path, &options))

        let asset = MDLAsset(url: url)
        let meshes = asset.childObjects(of: MDLMesh.self).compactMap { $0 as? MDLMesh }
        let loaded = try #require(meshes.first)
        #expect(meshes.count == 1)
        #expect(loaded.vertexCount == mesh.vertexCount)
        let expected = mesh.positions.map { Self.exported($0) }
        let lo = expected.reduce(SIMD3<Float>(repeating: .infinity)) { simd_min($0, $1) }
        let hi = expected.reduce(SIMD3<Float>(repeating: -.infinity)) { simd_max($0, $1) }
        let box = loaded.boundingBox
        print("ModelIO bounding box: \(box.minBounds) ... \(box.maxBounds) mm")
        #expect(simd_distance(box.minBounds, lo) < 0.01 && simd_distance(box.maxBounds, hi) < 0.01)
        #expect(abs((box.maxBounds - box.minBounds).y - 200) < 1)  // 0.1 m radius sphere = 200 mm across

        let submesh = try #require(loaded.submeshes?.firstObject as? MDLSubmesh)
        let property = try #require(submesh.material?.property(with: .baseColor))
        let reference: String
        switch property.type {
        case .URL: reference = property.urlValue?.lastPathComponent ?? ""
        case .string: reference = property.stringValue ?? ""
        case .texture: reference = property.textureSamplerValue?.texture?.name ?? property.urlValue?.lastPathComponent ?? ""
        default: reference = ""
        }
        print("ModelIO baseColor: type \(property.type.rawValue), \(reference)")
        #expect(reference.hasSuffix("head.png"))
        // Loading the textures resolves the reference relative to the OBJ: the PNG must load with its size.
        asset.loadTextures()
        let loadedTexture = try #require(submesh.material?.property(with: .baseColor)?.textureSamplerValue?.texture)
        #expect(loadedTexture.dimensions == SIMD2<Int32>(Int32(texture.width), Int32(texture.height)))
    }

    @Test func untexturedExportWritesNoMaterial() throws {
        var mesh = MeshFactory.icosphere(level: 2)
        mesh.colors = mesh.positions.map { _ in [255, 128, 0] }
        let dir = try TemporaryDirectory()
        let url = dir.url.appendingPathComponent("plain.obj")
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var options = Self.exportOptions(transform: matrix_identity_float4x4)
        options.scale = 1
        try check(fr_export_obj(core, nil, url.path, &options))
        let text = try String(contentsOf: url, encoding: .utf8)
        #expect(!text.contains("mtllib") && text.contains("# units: meters"))
        #expect(text.contains(" 1.0000 0.5020 0.0000\n"))
        #expect(try FileManager.default.contentsOfDirectory(atPath: dir.url.path) == ["plain.obj"])
        // "f a b c" without vt/vn
        #expect(text.split(separator: "\n").first { $0.hasPrefix("f ") }?.contains("/") == false)
    }

    @Test func mirroringTransformKeepsOutwardWinding() throws {
        let mesh = MeshFactory.icosphere(level: 2)
        let dir = try TemporaryDirectory()
        let url = dir.url.appendingPathComponent("mirror.obj")
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var options = Self.exportOptions(transform: simd_float4x4(diagonal: [1, -1, 1, 1]))
        try check(fr_export_obj(core, nil, url.path, &options))
        var imported: UnsafeMutablePointer<fr_mesh>?
        try check(fr_import_obj(url.path, &imported))
        let back = ReconMesh(core: try #require(imported))
        fr_mesh_free(imported)
        #expect(MeshMeasure.signedVolume(back) > 0)
    }

    @Test func plyLayout() throws {
        let (mesh, _) = try Self.texturedSphere()
        let dir = try TemporaryDirectory()
        let url = dir.url.appendingPathComponent("head.ply")
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var options = Self.exportOptions()
        try check(fr_export_ply(core, url.path, &options))
        let data = try Data(contentsOf: url)
        let headerEnd = try #require(data.range(of: Data("end_header\n".utf8)))
        let header = String(decoding: data[..<headerEnd.upperBound], as: UTF8.self)
        #expect(header.hasPrefix("ply\nformat binary_little_endian 1.0\n"))
        #expect(header.contains("element vertex \(mesh.vertexCount)\n") && header.contains("element face \(mesh.triangleCount)\n"))
        #expect(header.contains("property float nx\n") && header.contains("property uchar red\n")
            && header.contains("property float texture_u\n") && header.contains("property list uchar int vertex_indices\n"))
        let vertexBytes = 12 + 12 + 3 + 8
        #expect(data.count == headerEnd.upperBound + mesh.vertexCount * vertexBytes + mesh.triangleCount * 13)
        // First vertex and face decode back.
        let body = data[headerEnd.upperBound...]
        let x = body.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: 0, as: SIMD3<Float>.self) }
        #expect(simd_distance(x, Self.exported(mesh.positions[0])) < 1e-3)
        let faceStart = mesh.vertexCount * vertexBytes
        let face = body.withUnsafeBytes { raw -> (UInt8, Int32, Int32, Int32) in
            (raw[faceStart], raw.loadUnaligned(fromByteOffset: faceStart + 1, as: Int32.self),
             raw.loadUnaligned(fromByteOffset: faceStart + 5, as: Int32.self), raw.loadUnaligned(fromByteOffset: faceStart + 9, as: Int32.self))
        }
        #expect(face.0 == 3 && face.1 == Int32(mesh.indices[0]) && face.2 == Int32(mesh.indices[1]) && face.3 == Int32(mesh.indices[2]))
    }

    /// Both exporters refuse NaN positions (the binary PLY used to write them verbatim) and leave no file behind.
    @Test func nonFinitePositionsAreRefused() throws {
        var mesh = MeshFactory.icosphere(level: 1)
        mesh.positions[3].y = .nan
        let dir = try TemporaryDirectory()
        let core = try mesh.makeCoreMesh()
        defer { fr_mesh_free(core) }
        var options = Self.exportOptions()
        #expect(fr_export_ply(core, dir.url.appendingPathComponent("bad.ply").path, &options) == FR_ERR_INVALID_ARGUMENT)
        #expect(fr_export_obj(core, nil, dir.url.appendingPathComponent("bad.obj").path, &options) == FR_ERR_INVALID_ARGUMENT)
        #expect(try FileManager.default.contentsOfDirectory(atPath: dir.url.path).isEmpty)
    }

    @Test func importsForeignOBJ() throws {
        let text = """
        # quad with separate uv/normal indices, a pentagon with negative indices and vertex colors
        v 0 0 0 1 0 0
        v 1 0 0 0 1 0
        v 1 1 0 0 0 1
        v 0 1 0 1 1 1
        vt 0 0
        vt 1 0
        vt 1 1
        vt 0 1
        vn 0 0 1
        g quad
        f 1/1/1 2/2/1 3/3/1 4/4/1
        v 2 0 0 0.5 0.5 0.5
        f -4//1 -1//1 -3//1
        """
        let dir = try TemporaryDirectory()
        let url = dir.url.appendingPathComponent("foreign.obj")
        try text.write(to: url, atomically: true, encoding: .utf8)
        var imported: UnsafeMutablePointer<fr_mesh>?
        try check(fr_import_obj(url.path, &imported))
        let mesh = ReconMesh(core: try #require(imported))
        fr_mesh_free(imported)
        #expect(mesh.triangleCount == 3)
        #expect(mesh.normals.allSatisfy { $0 == [0, 0, 1] })
        #expect(mesh.colors.first == [255, 0, 0])
        // Distinct (v, vt, vn) corners become distinct vertices: 4 for the quad, 3 more for the last triangle.
        #expect(mesh.vertexCount == 7)
        #expect(mesh.positions[Int(mesh.indices[7])] == [2, 0, 0])

        try "v 0 0 0\nf 1 2 3\n".write(to: url, atomically: true, encoding: .utf8)
        #expect(fr_import_obj(url.path, &imported) == FR_ERR_INVALID_ARGUMENT)
        #expect(imported == nil)
        #expect(fr_import_obj(dir.url.appendingPathComponent("missing.obj").path, &imported) == FR_ERR_IO)
    }

    @Test(arguments: [1, 3, 4])
    func pngRoundTrip(channels: Int32) throws {
        let w = 37, h = 23
        let pixels = (0 ..< w * h * Int(channels)).map { UInt8(truncatingIfNeeded: $0 &* 7) }
        let dir = try TemporaryDirectory()
        let url = dir.url.appendingPathComponent("image.png")
        try check(fr_write_png(url.path, pixels, Int32(w), Int32(h), channels))
        let source = try #require(CGImageSourceCreateWithURL(url as CFURL, nil))
        let image = try #require(CGImageSourceCreateImageAtIndex(source, 0, nil))
        #expect(image.width == w && image.height == h)
        #expect(Self.pngColorType(url) == [1: 0, 3: 2, 4: 6][channels])
        // ImageIO may decode RGB as RGBX; compare the channels of each decoded pixel.
        let data = try #require(image.dataProvider?.data as Data?)
        let bytesPerPixel = image.bitsPerPixel / 8
        let first = [.noneSkipFirst, .first, .premultipliedFirst].contains(image.alphaInfo) ? 1 : 0
        var mismatches = 0
        for y in 0 ..< h {
            for x in 0 ..< w {
                for c in 0 ..< Int(channels) where data[y * image.bytesPerRow + x * bytesPerPixel + first + c] != pixels[(y * w + x) * Int(channels) + c] {
                    mismatches += 1
                }
            }
        }
        #expect(mismatches == 0)
        #expect(fr_write_png(url.path, pixels, Int32(w), Int32(h), 2) == FR_ERR_INVALID_ARGUMENT)
    }
}
