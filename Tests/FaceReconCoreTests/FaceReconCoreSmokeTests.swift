// License: Apache 2.0. See LICENSE file in root directory.

import FaceReconCore
import Testing

@Test func meshAllocationRoundTrip() throws {
    var mesh: UnsafeMutablePointer<fr_mesh>?
    #expect(fr_mesh_create(3, 1, FR_MESH_NORMALS | FR_MESH_COLORS, &mesh) == FR_OK)
    let m = try #require(mesh)
    #expect(m.pointee.vertex_count == 3)
    #expect(m.pointee.normals != nil && m.pointee.colors != nil && m.pointee.uvs == nil)
    var copy: UnsafeMutablePointer<fr_mesh>?
    #expect(fr_mesh_clone(m, &copy) == FR_OK)
    fr_mesh_free(copy)
    fr_mesh_free(m)
}
