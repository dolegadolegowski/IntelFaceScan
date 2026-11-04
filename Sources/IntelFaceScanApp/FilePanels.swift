// License: Apache 2.0. See LICENSE file in root directory.
//
// Open and save panels as async calls on the main actor.

import AppKit
import UniformTypeIdentifiers

@MainActor
enum FilePanels {
    static let db3 = UTType(filenameExtension: "db3") ?? .data
    static let bag = UTType(filenameExtension: "bag") ?? .data
    static let obj = UTType(filenameExtension: "obj") ?? .data
    static let ply = UTType(filenameExtension: "ply") ?? .data

    /// Asks for a destination; nil when cancelled. An empty `contentTypes` allows any name.
    static func save(suggestedName: String, contentTypes: [UTType]) async -> URL? {
        let panel = NSSavePanel()
        panel.nameFieldStringValue = suggestedName
        panel.canCreateDirectories = true
        panel.isExtensionHidden = false
        if !contentTypes.isEmpty { panel.allowedContentTypes = contentTypes }
        return await run(panel).map { normalizedSaveURL($0, contentTypes: contentTypes) }
    }

    /// The chosen URL with its extension once: the panel appends the extension of a type the system does not declare
    /// (a dynamic one, such as .db3) to a name that already ends with it, which saved "head-....db3.db3". The doubled
    /// name stays when the single one already exists (the panel asked about replacing the other name only).
    nonisolated static func normalizedSaveURL(_ url: URL, contentTypes: [UTType],
                                              exists: (URL) -> Bool = { FileManager.default.fileExists(atPath: $0.path) }) -> URL {
        var name = url.lastPathComponent
        for ext in Set(contentTypes.compactMap { $0.preferredFilenameExtension?.lowercased() }) {
            let doubled = ".\(ext).\(ext)"
            while name.lowercased().hasSuffix(doubled) { name.removeLast(ext.count + 1) }
        }
        let single = url.deletingLastPathComponent().appendingPathComponent(name)
        return single == url || exists(single) ? url : single
    }

    /// Asks for an existing file of one of the types.
    static func openFile(contentTypes: [UTType]) async -> URL? {
        let panel = NSOpenPanel()
        panel.canChooseFiles = true
        panel.canChooseDirectories = false
        panel.allowsMultipleSelection = false
        panel.allowedContentTypes = contentTypes
        return await run(panel)
    }

    /// Asks for a folder (e.g. a .ifsdataset session).
    static func openDirectory() async -> URL? {
        let panel = NSOpenPanel()
        panel.canChooseFiles = false
        panel.canChooseDirectories = true
        panel.allowsMultipleSelection = false
        panel.treatsFilePackagesAsDirectories = false
        return await run(panel)
    }

    private static func run(_ panel: NSSavePanel) async -> URL? {
        await withCheckedContinuation { continuation in
            let completion: (NSApplication.ModalResponse) -> Void = { response in
                continuation.resume(returning: response == .OK ? panel.url : nil)
            }
            if let window = NSApp.keyWindow ?? NSApp.mainWindow {
                panel.beginSheetModal(for: window, completionHandler: completion)
            } else {
                panel.begin(completionHandler: completion)
            }
        }
    }
}
