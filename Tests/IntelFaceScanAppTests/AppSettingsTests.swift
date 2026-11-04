// License: Apache 2.0. See LICENSE file in root directory.
//
// Settings: what they map to in the reconstruction options, and loading settings stored by an older version.

import Foundation
import ReconKit
import Testing
@testable import IntelFaceScanApp

@Suite("App settings")
struct AppSettingsTests {
    @Test("Closing the back of the head is on by default and reaches the reconstruction options")
    func closeHeadMapping() {
        var settings = AppSettings()
        #expect(settings.closeHead)
        #expect(settings.reconstructionOptions.closeHead)
        let closed = settings.reconstructionOptions
        settings.closeHead = false
        #expect(!settings.reconstructionOptions.closeHead)
        // The result screen offers to reconstruct again when the options differ from the ones the result used.
        #expect(settings.reconstructionOptions != closed)
    }

    @Test("Settings stored before the closing option existed load with it on")
    func olderSettingsKeepTheDefault() throws {
        let suite = "IntelFaceScanAppTests.settings.\(UUID().uuidString)"
        let defaults = try #require(UserDefaults(suiteName: suite))
        defer { defaults.removePersistentDomain(forName: suite) }
        var stored = try #require(try JSONSerialization.jsonObject(with: JSONEncoder().encode(AppSettings())) as? [String: Any])
        stored.removeValue(forKey: "closeHead")
        stored["keepShoulders"] = true
        defaults.set(try JSONSerialization.data(withJSONObject: stored), forKey: AppSettings.defaultsKey)

        let loaded = AppSettings.load(from: defaults)
        #expect(loaded.keepShoulders)
        #expect(loaded.closeHead)

        var off = loaded
        off.closeHead = false
        off.save(to: defaults)
        #expect(!AppSettings.load(from: defaults).closeHead)
    }
}
