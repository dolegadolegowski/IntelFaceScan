// License: Apache 2.0. See LICENSE file in root directory.
//
// User-visible strings in English and Polish: plural forms, the 3D preview's labels and titles, and that every
// value of the enumerations the UI names has its own Polish text.

import CameraKit
import Foundation
import ReconKit
import ScanKit
import simd
import Testing
@testable import IntelFaceScanApp

@Suite("App localization")
struct AppLocalizationTests {
    @Test("Polish plural forms: one, few (2-4 except 12-14), many")
    func polishPlural() {
        let expected: [(Int, String)] = [
            (0, "trójkątów"), (1, "trójkąt"), (2, "trójkąty"), (3, "trójkąty"), (4, "trójkąty"), (5, "trójkątów"),
            (11, "trójkątów"), (12, "trójkątów"), (13, "trójkątów"), (14, "trójkątów"), (21, "trójkątów"),
            (22, "trójkąty"), (24, "trójkąty"), (25, "trójkątów"), (101, "trójkątów"), (102, "trójkąty"),
            (112, "trójkątów"), (122, "trójkąty"), (1_001, "trójkątów"), (1_002, "trójkąty"), (823_904, "trójkąty"),
        ]
        for (n, word) in expected {
            #expect(S.polishPlural(n, "trójkąt", "trójkąty", "trójkątów") == word, "\(n)")
        }
    }

    @Test("Triangle counts read correctly in both languages")
    func triangleCounts() {
        let one = AppTest.inBothLanguages { S.triangles(1) }
        #expect(one.en == "1 triangle")
        #expect(one.pl == "1 trójkąt")
        let few = AppTest.inBothLanguages { S.triangles(22) }
        #expect(few.en == "22 triangles")
        #expect(few.pl == "22 trójkąty")
        let teen = AppTest.inBothLanguages { S.triangles(13) }
        #expect(teen.pl == "13 trójkątów")
        #expect(AppTest.inBothLanguages { S.triangles(5) }.pl == "5 trójkątów")
        #expect(AppTest.inBothLanguages { S.triangles(0) }.en == "0 triangles")
        // Large counts carry the locale's digit grouping; the noun still follows the count.
        let large = AppTest.inBothLanguages { S.triangles(823_904) }
        #expect(large.en.hasSuffix(" triangles"))
        #expect(large.pl.hasSuffix(" trójkąty"))
    }

    @Test("3D preview overlay: size, details and the fallback note")
    func previewOverlay() {
        let size = AppTest.inBothLanguages { S.modelSize(SIMD3(161.4, 230.6, 0.2)) }
        #expect(size.en == "161 × 231 × 0 mm (W × H × D)")
        #expect(size.pl == "161 × 231 × 0 mm (szer. × wys. × gł.)")

        // One triangle without texture or colors: the overlay the result screen showed untranslated.
        let mesh = ReconMesh(positions: [SIMD3(0, 0, 0), SIMD3(0.1, 0, 0), SIMD3(0, 0.1, 0)], indices: [0, 1, 2])
        let bare = ReconstructionResult(mesh: mesh, texture: nil, cameraPoses: [],
                                        exportTransform: matrix_identity_float4x4, report: ReconstructionReport())
        let detail = AppTest.inBothLanguages { ModelInfoLabel.detailText(bare) }
        #expect(detail.en == "1 triangle")
        #expect(detail.pl == "1 trójkąt")
        let note = AppTest.inBothLanguages { ModelInfoLabel.note(bare, displayMode: .texture) }
        #expect(note.en == "No texture: showing shading")
        #expect(note.pl == "Brak tekstury: pokazano cieniowanie")
        let colorNote = AppTest.inBothLanguages { ModelInfoLabel.note(bare, displayMode: .vertexColors) }
        #expect(colorNote.pl == "Brak kolorów wierzchołków: pokazano cieniowanie")
        #expect(ModelInfoLabel.note(bare, displayMode: .shaded) == nil)
        #expect(ModelInfoLabel.note(bare, displayMode: .wireframe) == nil)

        // Colors but no texture: the texture mode falls back to the vertex colors, as the scene does.
        var colored = mesh
        colored.colors = [SIMD3(255, 0, 0), SIMD3(0, 255, 0), SIMD3(0, 0, 255)]
        let withColors = ReconstructionResult(mesh: colored, texture: TextureImage(width: 2, height: 2, rgba: Data(count: 16)),
                                              cameraPoses: [], exportTransform: matrix_identity_float4x4,
                                              report: ReconstructionReport())
        #expect(AppTest.inBothLanguages { ModelInfoLabel.note(withColors, displayMode: .texture) }.pl
                == "Brak tekstury: pokazano kolory wierzchołków")
        #expect(ModelInfoLabel.note(withColors, displayMode: .vertexColors) == nil)
        let textured = AppTest.inBothLanguages { ModelInfoLabel.detailText(withColors) }
        #expect(textured.en == "1 triangle · texture 2×2")
        #expect(textured.pl == "1 trójkąt · tekstura 2×2")
    }

    @Test("Display mode titles are localized")
    func displayModeTitles() {
        let titles = AppTest.inBothLanguages { ModelDisplayMode.allCases.map(\.title) }
        #expect(titles.en == ["Texture", "Vertex colors", "Shaded", "Wireframe"])
        #expect(titles.pl == ["Tekstura", "Kolory wierzchołków", "Cieniowanie", "Siatka"])
    }

    /// Every value has a Polish text of its own (not the English one, not empty) and the texts are distinct.
    private func expectTranslated<T>(_ values: [T], _ name: String, allowSame: Set<String> = [],
                                     _ text: (T) -> String) {
        let both = AppTest.inBothLanguages { values.map(text) }
        for (en, pl) in zip(both.en, both.pl) {
            #expect(!pl.isEmpty, "\(name): empty Polish text for \(en)")
            #expect(pl != en || allowSame.contains(en), "\(name): \"\(en)\" has no Polish translation")
        }
        #expect(Set(both.pl).count == both.pl.count, "\(name): duplicate Polish texts \(both.pl)")
    }

    @Test("Enumerations shown in the UI have Polish texts")
    func enumerationsTranslated() {
        expectTranslated(ScanPose.allCases, "poseName", S.poseName)
        expectTranslated(ScanPose.allCases, "poseInstruction", S.poseInstruction)
        expectTranslated(ReconstructionProgress.Stage.allCases, "stageName", S.stageName)
        expectTranslated(ModelDisplayMode.allCases, "displayModeName", S.displayModeName)
        expectTranslated(AppSettings.PreviewMode.allCases, "previewMode") { $0.label }
        expectTranslated(AppSettings.DepthPreset.allCases, "preset") { $0.label }
        expectTranslated(ScanRejection.allCases, "rejection", S.rejection)
        let timings = ["segmentation", "registration", "registration check", "bounds", "fusion", "poisson", "cleanup",
                       "texturing"]
        expectTranslated(timings, "timingName", S.timingName)
    }

    @Test("Messages built in the app are localized")
    func messages() {
        let exit = AppTest.inBothLanguages { S.exitStatus(3) }
        #expect(exit.en == "Exit status 3")
        #expect(exit.pl == "Kod wyjścia 3")
        let few = AppTest.inBothLanguages { S.reconstructionError(ReconstructionError.notEnoughKeyframes(2)) }
        #expect(few.en.hasPrefix("Not enough usable frames (2)."))
        #expect(few.pl.hasPrefix("Za mało użytecznych klatek (2)."))
        let core = AppTest.inBothLanguages {
            S.reconstructionError(ReconstructionError.core(stage: "fusion", status: -1, message: "no volume"))
        }
        #expect(core.pl == "Etap „fusion” nie powiódł się: no volume")
        struct Other: LocalizedError { var errorDescription: String? { "disk full" } }
        #expect(S.reconstructionError(Other()) == "disk full")
        let frames = AppTest.inBothLanguages { S.scanSummary(22, 5) }
        #expect(frames.en == "22 frames from 5 poses captured.")
        #expect(frames.pl == "Zarejestrowano 22 klatki z 5 póz.")
        #expect(AppTest.inBothLanguages { S.keyframes(1) }.pl == "1 klatka")
        #expect(AppTest.inBothLanguages { S.keyframes(14) }.pl == "14 klatek")
        #expect(AppTest.inBothLanguages { S.resetView }.pl == "Przywróć widok")
    }

    /// The details ReconstructionPipeline and the core report today (ReconstructionPipeline.emit, the core's
    /// registration and Poisson progress); English passes through, Polish never shows English.
    @Test("Reconstruction progress details")
    func progressDetails() {
        let sent = ["15 keyframes", "head masks", "14 usable keyframes", "building clouds", "pairwise registration",
                    "pose graph optimization", "registration finished", "registration finished (some frames unregistered)",
                    "checking registration", "integrating frame 3 of 15", "marching cubes", "merging registered clouds",
                    "screened Poisson (depth 10)", "poisson: solving", "poisson: trimming", "poisson: finished", "cleanup",
                    "texturing", "412118 vertices, 823904 triangles"]
        for message in sent {
            let both = AppTest.inBothLanguages { S.progressDetail(message) }
            #expect(both.en == message)
            #expect(!both.pl.isEmpty, "\"\(message)\" has no Polish text")
            #expect(both.pl != message || message == "marching cubes", "\"\(message)\" is not translated")
        }
        let pl = { (m: String) in AppLanguage.$override.withValue(.polish) { S.progressDetail(m) } }
        #expect(pl("1 keyframes") == "1 klatka")
        #expect(pl("22 usable keyframes") == "22 użyteczne klatki")
        #expect(pl("integrating frame 3 of 15") == "integracja klatki 3 z 15")
        #expect(pl("2 vertices, 5 triangles") == "2 wierzchołki, 5 trójkątów")
        #expect(pl("some new English detail") == "", "unknown details leak into the Polish UI")
        #expect(pl("") == "")
    }

    /// The notes ReconstructionPipeline writes into the report (shown under the result's report): Polish for every
    /// one of them; English passes through, and a note the app does not know stays in English instead of vanishing.
    @Test("Reconstruction report notes")
    func reportNotes() {
        let written = [
            "3 keyframes with inconsistent image data skipped",
            "1 keyframes without usable head depth skipped",
            "initial poses: 14 landmark fits, 0 Vision rotation + landmark translation, 0 Vision rotation + head centroid",
            "keyframes excluded after registration (surface does not fit): 7, 12",
            String(format: "volume too large for %.2f mm voxels; using %.2f mm", 0.5, 0.62),
            "no depth at the chin: neck not cropped",
            "Metal unavailable: fused on the CPU",
            "chin crop: triangle-level fallback (fr_mesh_crop_plane not implemented)",
            "mesh cleanup failed: invalid argument",
            "vertex colors failed: no views",
            "texture failed: no views",
        ]
        for note in written {
            let both = AppTest.inBothLanguages { S.reportNote(note) }
            #expect(both.en == note)
            #expect(both.pl != note, "\"\(note)\" is not translated")
            #expect(!both.pl.isEmpty)
        }
        let pl = { (n: String) in AppLanguage.$override.withValue(.polish) { S.reportNote(n) } }
        #expect(pl("1 keyframes without usable head depth skipped") == "pominięto 1 klatkę bez użytecznej głębi głowy")
        #expect(pl("3 keyframes with inconsistent image data skipped") == "pominięto 3 klatki z niespójnymi danymi obrazu")
        #expect(pl("12 keyframes with inconsistent image data skipped") == "pominięto 12 klatek z niespójnymi danymi obrazu")
        #expect(pl("keyframes excluded after registration (surface does not fit): 7, 12")
                == "klatki wykluczone po rejestracji (powierzchnia nie pasuje): 7, 12")
        #expect(pl("volume too large for 0.50 mm voxels; using 0.62 mm") == "objętość za duża dla wokseli 0.50 mm; użyto 0.62 mm")
        #expect(pl("texture failed: no views") == "błąd kroku „tekstura”: no views")
        #expect(pl("a note added later") == "a note added later", "an unknown note must still show")
    }

    @Test("The override is scoped to the task")
    func overrideScope() async {
        let outside = AppLanguage.current
        await AppLanguage.$override.withValue(.polish) {
            #expect(AppLanguage.current == .polish)
            await Task.yield()
            #expect(S.cancel == "Anuluj")
        }
        #expect(AppLanguage.current == outside)
    }
}
