// License: Apache 2.0. See LICENSE file in root directory.

import Foundation
import ModelIO
import ReconKit
import simd
import Testing

@Suite("MeshExport")
struct ExportTests {
    /// Textured sphere 0.5 m in front of the camera, exported upright with the origin at its center.
    static func result(textured: Bool = true) throws -> ReconstructionResult {
        var mesh = TestMeshes.icosphere(level: 3, center: [0, 0, 0.5])
        let scene = SphereScene.keyframes()
        let poses = scene.poses.map { pose -> simd_float4x4 in
            var p = pose
            p.columns.3.z += 0.5
            return p
        }
        var options = ReconstructionOptions()
        options.textureSize = 256
        var texture: TextureImage?
        if textured {
            texture = try Texturing.unwrapAndBake(&mesh, keyframes: scene.keyframes, poses: poses, options: options)
        }
        try Texturing.colorizeVertices(&mesh, keyframes: scene.keyframes, poses: poses, options: options)
        // Camera frame (+Y down, +Z forward) -> export frame (+Y up, face toward +Z): rotate 180 degrees about X.
        var exportTransform = simd_float4x4(simd_quatf(angle: .pi, axis: [1, 0, 0]))
        exportTransform.columns.3 = [0, 0, 0.5, 1]
        return ReconstructionResult(mesh: mesh, texture: texture, cameraPoses: poses, exportTransform: exportTransform,
                                    report: ReconstructionReport())
    }

    @Test func writesTexturedOBJThatModelIOLoads() throws {
        let result = try Self.result()
        let dir = try ScratchDirectory()
        let files = try MeshExport.writeOBJ(result, to: dir.url.appendingPathComponent("My Head.obj"))
        #expect(files.map(\.lastPathComponent) == ["My Head.obj", "My_Head.mtl", "My_Head.png"])
        #expect(files.allSatisfy { FileManager.default.fileExists(atPath: $0.path) })

        let asset = MDLAsset(url: files[0])
        let mesh = try #require(asset.childObjects(of: MDLMesh.self).first as? MDLMesh)
        #expect(mesh.vertexCount == result.mesh.vertexCount)
        let box = mesh.boundingBox
        print("MeshExport bounding box: \(box.minBounds) ... \(box.maxBounds)")
        // Centered, 200 mm across.
        #expect(simd_length(box.minBounds + box.maxBounds) < 1)
        #expect(simd_reduce_max(simd_abs(box.maxBounds - box.minBounds - SIMD3(repeating: 200))) < 2)
        asset.loadTextures()
        let submesh = try #require(mesh.submeshes?.firstObject as? MDLSubmesh)
        #expect(submesh.material?.property(with: .baseColor)?.textureSamplerValue?.texture?.dimensions == [256, 256])
    }

    @Test func optionsControlTheContent() throws {
        let result = try Self.result()
        let dir = try ScratchDirectory()
        var options = ExportOptions()
        options.includeTexture = false
        options.includeNormals = false
        options.includeVertexColors = false
        options.unitsScale = 1
        let files = try MeshExport.writeOBJ(result, to: dir.url.appendingPathComponent("bare.obj"), options: options)
        #expect(files.count == 1)
        let text = try String(contentsOf: files[0], encoding: .utf8)
        #expect(text.contains("# units: meters") && !text.contains("\nvt ") && !text.contains("\nvn ") && !text.contains("mtllib"))
        let firstVertex = try #require(text.split(separator: "\n").first { $0.hasPrefix("v ") })
        #expect(firstVertex.split(separator: " ").count == 4)  // no colors
    }

    @Test func writesPLY() throws {
        let result = try Self.result(textured: false)
        let dir = try ScratchDirectory()
        let url = try MeshExport.writePLY(result, to: dir.url.appendingPathComponent("head.ply"))
        let data = try Data(contentsOf: url)
        let header = try #require(String(data: data.prefix(600), encoding: .isoLatin1))
        #expect(header.contains("element vertex \(result.mesh.vertexCount)\n") && header.contains("comment units: millimeters\n"))
        let end = try #require(data.range(of: Data("end_header\n".utf8)))
        #expect(result.mesh.normals.isEmpty && result.mesh.colors.count == result.mesh.vertexCount)
        #expect(!header.contains("property float nx") && header.contains("property uchar red"))
        #expect(data.count == end.upperBound + result.mesh.vertexCount * (12 + 3) + result.mesh.triangleCount * 13)
    }

    @Test func refusesEmptyMeshes() {
        let empty = ReconstructionResult(mesh: ReconMesh(), texture: nil, cameraPoses: [],
                                         exportTransform: matrix_identity_float4x4, report: ReconstructionReport())
        #expect(throws: ReconstructionError.self) {
            try MeshExport.writeOBJ(empty, to: FileManager.default.temporaryDirectory.appendingPathComponent("empty.obj"))
        }
    }
}
