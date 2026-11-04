// License: Apache 2.0. See LICENSE file in root directory.
//
// Metal renderer of the live preview: color, turbo-colored depth or both blended, optionally mirrored.
// Frames are copied into GPU buffers on the frame thread (submit); the main thread only encodes a draw of the
// newest uploaded frame, so preview cost on the main actor does not depend on the frame size.
// Shaders are compiled at runtime from source (the Command Line Tools have no offline Metal compiler).

import CameraKit
import Foundation
import Metal
import MetalKit
import QuartzCore
import ScanKit

struct PreviewDisplay: Equatable {
    var mode: AppSettings.PreviewMode = .color
    var mirrored = true
    /// Depth range mapped onto the colormap (meters).
    var nearM: Float = 0.3
    var farM: Float = 1.2
    var blend: Float = 0.5
}

enum PreviewRendererError: Error, LocalizedError {
    case noDevice
    case setup(String)

    var errorDescription: String? {
        switch self {
        case .noDevice: return "No Metal device."
        case .setup(let m): return m
        }
    }
}

final class PreviewRenderer: NSObject, MTKViewDelegate, @unchecked Sendable {
    let device: MTLDevice
    private let queue: MTLCommandQueue
    private let pipeline: MTLRenderPipelineState

    /// One uploaded frame. Buffers are reused while the frame size stays the same.
    private final class Slot {
        var color: MTLBuffer?
        var depth: MTLBuffer?
        var width = 0, height = 0
        var hasDepth = false
        var depthScale: Float = 0.001
        var sequence: UInt64 = 0
    }

    private let lock = NSLock()
    private let slots = [Slot(), Slot(), Slot()]
    private var latest: Int?
    private var writing: Int?
    private var inFlight: [Int: Int] = [:]          // slot -> number of command buffers using it
    private var display = PreviewDisplay()
    private var redrawPending = false
    private weak var view: MTKView?

    /// - Parameter device: nil = the system default Metal device.
    init(device preferred: MTLDevice?) throws {
        guard let device = preferred ?? MTLCreateSystemDefaultDevice() else { throw PreviewRendererError.noDevice }
        guard let queue = device.makeCommandQueue() else { throw PreviewRendererError.setup("No Metal command queue.") }
        self.device = device
        self.queue = queue
        do {
            let library = try device.makeLibrary(source: Self.shaderSource, options: nil)
            let descriptor = MTLRenderPipelineDescriptor()
            descriptor.vertexFunction = library.makeFunction(name: "preview_vertex")
            descriptor.fragmentFunction = library.makeFunction(name: "preview_fragment")
            descriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
            pipeline = try device.makeRenderPipelineState(descriptor: descriptor)
        } catch {
            throw PreviewRendererError.setup("Preview shaders: \(error.localizedDescription)")
        }
        super.init()
    }

    // MARK: - Main thread

    @MainActor
    func attach(_ view: MTKView) {
        view.device = device
        view.delegate = self
        view.colorPixelFormat = .bgra8Unorm
        view.framebufferOnly = true
        view.isPaused = true
        view.enableSetNeedsDisplay = true
        view.autoResizeDrawable = true
        view.clearColor = MTLClearColor(red: 0.05, green: 0.05, blue: 0.06, alpha: 1)
        view.colorspace = CGColorSpace(name: CGColorSpace.sRGB)
        self.view = view
        view.needsDisplay = true
    }

    @MainActor
    func setDisplay(_ newValue: PreviewDisplay) {
        let changed = lock.withLock { () -> Bool in
            guard display != newValue else { return false }
            display = newValue
            return true
        }
        if changed { view?.needsDisplay = true }
    }

    /// Drops the current image (source closed). Any thread: the redraw goes to the main thread.
    func clear() {
        lock.withLock { latest = nil }
        requestRedraw()
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        let (index, settings): (Int?, PreviewDisplay) = lock.withLock {
            if let latest { inFlight[latest, default: 0] += 1 }
            return (latest, display)
        }
        guard let descriptor = view.currentRenderPassDescriptor, let drawable = view.currentDrawable,
              let commands = queue.makeCommandBuffer(),
              let encoder = commands.makeRenderCommandEncoder(descriptor: descriptor) else {
            if let index { release(index) }
            return
        }
        if let index {
            encodeImage(slots[index], settings: settings, into: encoder)
            commands.addCompletedHandler { [weak self] _ in self?.release(index) }
        }
        encoder.endEncoding()
        commands.present(drawable)
        commands.commit()
    }

    private func encodeImage(_ slot: Slot, settings: PreviewDisplay, into encoder: MTLRenderCommandEncoder) {
        guard let color = slot.color else { return }
        var uniforms = Uniforms(width: UInt32(slot.width), height: UInt32(slot.height),
                                colorStride: UInt32(slot.width * 3),
                                mode: slot.hasDepth ? Self.modeIndex(settings.mode) : 0,
                                mirror: settings.mirrored ? 1 : 0, hasDepth: slot.hasDepth ? 1 : 0,
                                depthScale: slot.depthScale, nearM: settings.nearM,
                                farM: max(settings.farM, settings.nearM + 0.01), blend: settings.blend)
        encoder.setRenderPipelineState(pipeline)
        encoder.setFragmentBytes(&uniforms, length: MemoryLayout<Uniforms>.stride, index: 0)
        encoder.setFragmentBuffer(color, offset: 0, index: 1)
        encoder.setFragmentBuffer(slot.depth ?? color, offset: 0, index: 2)
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
    }

    /// Renders a frame into an offscreen BGRA image with the same shaders (snapshots and shader checks).
    func renderOffscreen(_ frame: CameraFrame, display settings: PreviewDisplay, width: Int, height: Int) -> CGImage? {
        guard frame.color.isValidRGB8, width > 0, height > 0 else { return nil }
        let slot = Slot()
        guard upload(frame, into: slot) else { return nil }
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm, width: width, height: height,
                                                                  mipmapped: false)
        descriptor.usage = [.renderTarget]
        descriptor.storageMode = .shared
        guard let texture = device.makeTexture(descriptor: descriptor), let commands = queue.makeCommandBuffer() else {
            return nil
        }
        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = texture
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].storeAction = .store
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        guard let encoder = commands.makeRenderCommandEncoder(descriptor: pass) else { return nil }
        encodeImage(slot, settings: settings, into: encoder)
        encoder.endEncoding()
        commands.commit()
        commands.waitUntilCompleted()
        guard commands.status == .completed else { return nil }
        var bytes = [UInt8](repeating: 0, count: width * height * 4)
        texture.getBytes(&bytes, bytesPerRow: width * 4, from: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0)
        guard let provider = CGDataProvider(data: Data(bytes) as CFData) else { return nil }
        return CGImage(width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: width * 4,
                       space: CGColorSpace(name: CGColorSpace.sRGB) ?? CGColorSpaceCreateDeviceRGB(),
                       bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue
                                                    | CGBitmapInfo.byteOrder32Little.rawValue),
                       provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
    }

    // MARK: - Frame thread

    /// Uploads a frame (any thread). Drops it when every slot is busy.
    func submit(_ frame: CameraFrame) {
        let color = frame.color
        guard color.isValidRGB8 else { return }
        let index: Int? = lock.withLock {
            guard writing == nil else { return nil }
            let free = slots.indices.first { $0 != latest && inFlight[$0, default: 0] == 0 }
            writing = free
            return free
        }
        guard let index else { return }
        let slot = slots[index]
        let uploaded = upload(frame, into: slot)
        lock.withLock {
            writing = nil
            if uploaded { latest = index }
        }
        if uploaded { requestRedraw() }
    }

    private func upload(_ frame: CameraFrame, into slot: Slot) -> Bool {
        let color = frame.color
        let w = color.width, h = color.height
        let colorBytes = w * h * 3
        if slot.color == nil || slot.width != w || slot.height != h {
            slot.color = device.makeBuffer(length: colorBytes, options: .storageModeShared)
            slot.depth = device.makeBuffer(length: w * h * 2, options: .storageModeShared)
            slot.width = w; slot.height = h
        }
        guard let colorBuffer = slot.color else { return false }
        color.pixels.withUnsafeBytes { raw in
            guard let src = raw.baseAddress else { return }
            let dst = colorBuffer.contents()
            if color.bytesPerRow == w * 3 {
                dst.copyMemory(from: src, byteCount: colorBytes)
            } else {
                for y in 0 ..< h { (dst + y * w * 3).copyMemory(from: src + y * color.bytesPerRow, byteCount: w * 3) }
            }
        }
        let depth = frame.alignedDepth
        slot.hasDepth = false
        if depth.width == w, depth.height == h, depth.pixels.count >= w * h * 2, let depthBuffer = slot.depth {
            depth.pixels.withUnsafeBytes { raw in
                guard let src = raw.baseAddress else { return }
                depthBuffer.contents().copyMemory(from: src, byteCount: w * h * 2)
                slot.hasDepth = true
            }
            slot.depthScale = depth.depthScale
        }
        slot.sequence = frame.sequence
        return true
    }

    private func release(_ index: Int) {
        lock.withLock {
            let n = inFlight[index, default: 0] - 1
            inFlight[index] = n > 0 ? n : nil
        }
    }

    /// Coalesces redraw requests: at most one pending hop to the main thread.
    private func requestRedraw() {
        let schedule = lock.withLock { () -> Bool in
            guard !redrawPending else { return false }
            redrawPending = true
            return true
        }
        guard schedule else { return }
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.lock.withLock { self.redrawPending = false }
            self.view?.needsDisplay = true
        }
    }

    // MARK: - Shaders

    private struct Uniforms {
        var width: UInt32
        var height: UInt32
        var colorStride: UInt32
        var mode: UInt32
        var mirror: UInt32
        var hasDepth: UInt32
        var depthScale: Float
        var nearM: Float
        var farM: Float
        var blend: Float
    }

    private static func modeIndex(_ mode: AppSettings.PreviewMode) -> UInt32 {
        switch mode {
        case .color: return 0
        case .depth: return 1
        case .blend: return 2
        }
    }

    // Layout of PreviewUniforms must match Uniforms above (ten 4-byte fields).
    private static let shaderSource = """
    #include <metal_stdlib>
    using namespace metal;

    struct VertexOut {
        float4 position [[position]];
        float2 uv;
    };

    struct PreviewUniforms {
        uint width;
        uint height;
        uint colorStride;
        uint mode;        // 0 color, 1 depth, 2 blend
        uint mirror;
        uint hasDepth;
        float depthScale;
        float nearM;
        float farM;
        float blend;
    };

    // One triangle covering the viewport; uv has a top-left origin like the image.
    vertex VertexOut preview_vertex(uint vid [[vertex_id]]) {
        const float2 p[3] = { float2(-1.0, -1.0), float2(3.0, -1.0), float2(-1.0, 3.0) };
        VertexOut out;
        out.position = float4(p[vid], 0.0, 1.0);
        out.uv = float2((p[vid].x + 1.0) * 0.5, (1.0 - p[vid].y) * 0.5);
        return out;
    }

    // Piecewise-linear approximation of the Turbo colormap (near = blue, far = red).
    static float3 turbo(float t) {
        const float3 c[9] = {
            float3(0.19, 0.07, 0.23), float3(0.27, 0.36, 0.84), float3(0.16, 0.65, 0.99),
            float3(0.10, 0.90, 0.71), float3(0.48, 0.99, 0.30), float3(0.84, 0.90, 0.21),
            float3(0.99, 0.62, 0.15), float3(0.88, 0.29, 0.04), float3(0.48, 0.02, 0.01)
        };
        float x = saturate(t) * 8.0;
        int i = min(int(x), 7);
        return mix(c[i], c[i + 1], x - float(i));
    }

    static float3 rgbAt(device const uchar *rgb, constant PreviewUniforms &u, uint x, uint y) {
        uint i = y * u.colorStride + x * 3;
        return float3(rgb[i], rgb[i + 1], rgb[i + 2]) / 255.0;
    }

    fragment float4 preview_fragment(VertexOut in [[stage_in]],
                                     constant PreviewUniforms &u [[buffer(0)]],
                                     device const uchar *rgb [[buffer(1)]],
                                     device const ushort *depth [[buffer(2)]]) {
        float2 uv = in.uv;
        if (u.mirror != 0) { uv.x = 1.0 - uv.x; }
        float w = float(u.width), h = float(u.height);

        // Bilinear color.
        float fx = clamp(uv.x * w - 0.5, 0.0, w - 1.0);
        float fy = clamp(uv.y * h - 0.5, 0.0, h - 1.0);
        uint x0 = uint(fx), y0 = uint(fy);
        uint x1 = min(x0 + 1, u.width - 1), y1 = min(y0 + 1, u.height - 1);
        float ax = fx - float(x0), ay = fy - float(y0);
        float3 top = mix(rgbAt(rgb, u, x0, y0), rgbAt(rgb, u, x1, y0), ax);
        float3 bottom = mix(rgbAt(rgb, u, x0, y1), rgbAt(rgb, u, x1, y1), ax);
        float3 color = mix(top, bottom, ay);
        if (u.mode == 0 || u.hasDepth == 0) { return float4(color, 1.0); }

        // Nearest depth (interpolating across a silhouette would invent surfaces).
        uint xn = min(uint(uv.x * w), u.width - 1);
        uint yn = min(uint(uv.y * h), u.height - 1);
        ushort raw = depth[yn * u.width + xn];
        bool valid = raw != 0;
        float z = float(raw) * u.depthScale;
        float3 depthColor = valid ? turbo((z - u.nearM) / (u.farM - u.nearM)) : float3(0.06, 0.06, 0.07);
        if (u.mode == 1) { return float4(depthColor, 1.0); }
        return float4(mix(color, depthColor, valid ? u.blend : 0.0), 1.0);
    }
    """
}
