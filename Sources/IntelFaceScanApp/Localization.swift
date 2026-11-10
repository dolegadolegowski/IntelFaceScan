// License: Apache 2.0. See LICENSE file in root directory.
//
// User-facing strings in English and Polish. The package cannot ship resources, so the strings live in code and
// the language is chosen once from Locale.preferredLanguages (the first supported language wins; launch with
// `-AppleLanguages '(pl)'` to see the Polish UI). Tests pick a language per task through AppLanguage.override.

import CameraKit
import Foundation
import ReconKit
import ScanKit

enum AppLanguage: Sendable {
    case english, polish

    static let preferred: AppLanguage = {
        for identifier in Locale.preferredLanguages {
            let code = identifier.lowercased()
            if code.hasPrefix("pl") { return .polish }
            if code.hasPrefix("en") { return .english }
        }
        return .english
    }()

    /// Replaces the preferred language within a scope (`AppLanguage.$override.withValue(.polish) { ... }`).
    @TaskLocal static var override: AppLanguage?

    static var current: AppLanguage { override ?? preferred }
}

/// Picks the string of the current language.
@inline(__always)
func tr(_ english: String, _ polish: String) -> String {
    AppLanguage.current == .polish ? polish : english
}

enum S {
    // MARK: App and sources
    static var appName: String { "IntelFaceScan" }
    static var sources: String { tr("Source", "Źródło") }
    static var liveCamera: String { tr("Live camera", "Kamera na żywo") }
    static var liveCameraHint: String {
        tr("Intel RealSense D455 over USB-C. The first use installs the RealSense camera service (an administrator's "
           + "name and password, once); after that no password is needed.",
           "Intel RealSense D455 przez USB-C. Przy pierwszym użyciu instalowana jest usługa kamery RealSense (nazwa "
           + "i hasło administratora, jeden raz); potem hasło nie jest potrzebne.")
    }
    static var openRecording: String { tr("Open recording…", "Otwórz nagranie…") }
    static var openRecordingHint: String { tr("Play a .db3 or .bag recording.", "Odtwórz nagranie .db3 lub .bag.") }
    static var openDataset: String { tr("Open dataset…", "Otwórz zestaw danych…") }
    static var openDatasetHint: String { tr("Replay a saved scan session (.ifsdataset).", "Odtwórz zapisaną sesję skanu (.ifsdataset).") }
    static var syntheticDemo: String { tr("Synthetic demo", "Demo syntetyczne") }
    static var syntheticDemoHint: String { tr("A rendered head, no camera needed.", "Renderowana głowa, bez kamery.") }
    static var closeSource: String { tr("Close source", "Zamknij źródło") }
    static var connecting: String { tr("Connecting…", "Łączenie…") }
    static var waitingForPassword: String {
        tr("Waiting for the administrator password…", "Oczekiwanie na hasło administratora…")
    }
    static var checkingCameraService: String {
        tr("Checking the RealSense camera service…", "Sprawdzanie usługi kamery RealSense…")
    }
    static var installingCameraService: String {
        tr("Installing the RealSense camera service: macOS asks for an administrator's name and password…",
           "Instalowanie usługi kamery RealSense: macOS prosi o nazwę i hasło administratora…")
    }
    static var startingCamera: String { tr("Starting the camera…", "Uruchamianie kamery…") }
    static func connectionStage(_ stage: ConnectionStage) -> String {
        switch stage {
        case .connecting: return connecting
        case .checkingCameraService: return checkingCameraService
        case .installingCameraService: return installingCameraService
        case .startingCamera: return startingCamera
        case .waitingForPassword: return waitingForPassword
        }
    }
    static var welcomeTitle: String { tr("3D head scan", "Skan 3D głowy") }
    static var welcomeSubtitle: String {
        tr("Choose a source to start. The scan guides you through front, up, down, left and right.",
           "Wybierz źródło, aby zacząć. Skan poprowadzi Cię przez pozy: prosto, góra, dół, lewo i prawo.")
    }
    static func sourceName(_ kind: CameraSourceKind) -> String {
        switch kind {
        case .live: return liveCamera
        case .playback(let url): return tr("Recording: ", "Nagranie: ") + url.lastPathComponent
        case .dataset(let url): return tr("Dataset: ", "Zestaw danych: ") + url.lastPathComponent
        case .synthetic: return syntheticDemo
        }
    }

    // MARK: Camera service (docs/CAMERA-SERVICE.md)
    /// The text of the macOS administrator prompt that installs the service.
    static var cameraServiceInstallPrompt: String {
        tr("IntelFaceScan wants to install the RealSense camera service. This is needed once: afterwards the camera "
           + "works without a password, in every application that uses the service.",
           "IntelFaceScan chce zainstalować usługę kamery RealSense. Wystarczy to zrobić raz: potem kamera działa "
           + "bez hasła w każdej aplikacji, która korzysta z tej usługi.")
    }
    static var cameraServiceNotInstalledTitle: String {
        tr("The RealSense camera service was not installed", "Usługa kamery RealSense nie została zainstalowana")
    }
    static var cameraServiceInstallCancelled: String {
        tr("The administrator prompt was cancelled. With the service the camera works without a password; installing "
           + "it takes an administrator's name and password, once.",
           "Prośba o hasło administratora została anulowana. Z usługą kamera działa bez hasła; jej instalacja "
           + "wymaga podania nazwy i hasła administratora, jeden raz.")
    }
    static var cameraServiceInstallFailedTitle: String {
        tr("Could not install the RealSense camera service", "Nie można zainstalować usługi kamery RealSense")
    }
    /// The installer's own message stays as it is (English).
    static func cameraServiceInstallFailed(_ detail: String) -> String {
        tr("The installer reported: \(detail)", "Instalator zgłosił: \(detail)")
    }
    static var cameraServiceUntrustedTitle: String {
        tr("The RealSense camera service cannot be installed from this copy of the app",
           "Usługi kamery RealSense nie można zainstalować z tej kopii aplikacji")
    }
    static func cameraServiceUntrusted(_ detail: String) -> String {
        tr("An administrator has to move IntelFaceScan to the Applications folder first: the service is only "
           + "installed from a copy that other users cannot change.\n\n\(detail)",
           "Administrator musi najpierw przenieść IntelFaceScan do folderu Aplikacje: usługa jest instalowana tylko "
           + "z kopii, której inni użytkownicy nie mogą zmienić.\n\n\(detail)")
    }
    static var cameraServiceSwitchedOffTitle: String {
        tr("The RealSense camera service is switched off", "Usługa kamery RealSense jest wyłączona")
    }
    static var cameraServiceSwitchedOff: String {
        tr("It was switched off in System Settings > General > Login Items & Extensions. An administrator can switch "
           + "it on again there.",
           "Została wyłączona w Ustawieniach systemowych > Ogólne > Rzeczy otwierane podczas logowania "
           + "i rozszerzenia. Administrator może ją tam ponownie włączyć.")
    }
    static var cameraServiceNotConsoleUserTitle: String {
        tr("The RealSense camera service only serves the user logged in at this Mac's screen",
           "Usługa kamery RealSense obsługuje tylko użytkownika zalogowanego przy ekranie tego Maca")
    }
    static var cameraServiceNotConsoleUser: String {
        tr("This session is not the one at the screen (for example a remote login, or a user switched to the "
           + "background), so the camera cannot be used without a password here.",
           "Ta sesja nie jest sesją przy ekranie (np. logowanie zdalne albo użytkownik przełączony w tło), więc "
           + "kamery nie można tu używać bez hasła.")
    }
    static var cameraDisabledTitle: String {
        tr("Camera use is disabled on this Mac", "Korzystanie z kamer jest na tym Macu wyłączone")
    }
    static var cameraDisabled: String {
        tr("The administrator of this Mac has disabled cameras (device management or Screen Time). IntelFaceScan "
           + "cannot use the RealSense camera until that changes.",
           "Administrator tego Maca wyłączył kamery (zarządzanie urządzeniem lub Czas przed ekranem). IntelFaceScan "
           + "nie może używać kamery RealSense, dopóki to się nie zmieni.")
    }
    static var cameraServiceBusyTitle: String {
        tr("The RealSense camera service is busy", "Usługa kamery RealSense jest zajęta")
    }
    static var cameraServiceBusy: String {
        tr("Too many programs are connected to it. Quit another program that uses a RealSense camera and try again.",
           "Połączonych jest z nią zbyt wiele programów. Zamknij inny program, który używa kamery RealSense, "
           + "i spróbuj ponownie.")
    }
    static var cameraServiceNotAnsweringTitle: String {
        tr("The RealSense camera service cannot be used", "Usługi kamery RealSense nie można używać")
    }
    static func cameraServiceStillNotUsable(_ state: String) -> String {
        tr("The installation finished, but the service cannot be used yet: \(state).",
           "Instalacja się zakończyła, ale usługi nie można jeszcze używać: \(state).")
    }
    /// The state as the end of a sentence ("State: ready (version 2).").
    static func cameraServiceState(_ availability: CameraService.Availability) -> String {
        switch availability {
        case .noKit:
            return tr("this copy of IntelFaceScan carries no service files",
                      "ta kopia IntelFaceScan nie zawiera plików usługi")
        case .state(let state):
            switch state {
            case .ready(let version):
                guard let version else { return tr("ready", "gotowa") }
                return tr("ready (version \(version))", "gotowa (wersja \(version))")
            case .notInstalled:
                return tr("not installed", "niezainstalowana")
            case .switchedOff:
                return tr("switched off in System Settings", "wyłączona w Ustawieniach systemowych")
            case .needsUpdate:
                return tr("installed, too old for this version of IntelFaceScan",
                          "zainstalowana, zbyt stara dla tej wersji IntelFaceScan")
            case .notAnswering(let detail):
                // The probe's own line stays as it is (English).
                return tr("installed, not answering", "zainstalowana, nie odpowiada")
                    + (detail.isEmpty ? "" : " (\(detail))")
            case .notConsoleUser:
                return tr("running, but it only serves the user logged in at this Mac's screen",
                          "działa, ale obsługuje tylko użytkownika zalogowanego przy ekranie tego Maca")
            case .cameraDisabled:
                return tr("camera use is disabled on this Mac", "korzystanie z kamer jest na tym Macu wyłączone")
            case .tooManyClients:
                return tr("too many programs are connected", "połączonych jest zbyt wiele programów")
            }
        }
    }
    static var cameraServiceMenu: String { tr("RealSense Camera Service…", "Usługa kamery RealSense…") }
    static var cameraServiceTitle: String { tr("RealSense camera service", "Usługa kamery RealSense") }
    /// The status command's text: the state, the installed version and the version this copy of the app carries.
    static func cameraServiceStatus(state: String, installed: Int?, bundled: Int?) -> String {
        var lines = [tr("State: \(state).", "Stan: \(state).")]
        if let installed {
            lines.append(tr("Installed version: \(installed).", "Zainstalowana wersja: \(installed)."))
        }
        if let bundled {
            lines.append(tr("Version in this copy of IntelFaceScan: \(bundled).",
                            "Wersja w tej kopii IntelFaceScan: \(bundled)."))
        }
        lines.append("")
        lines.append(tr("With the service, every application that uses it reaches RealSense cameras without a "
                        + "password. It is installed once, with an administrator's name and password.",
                        "Dzięki usłudze każda aplikacja, która z niej korzysta, ma dostęp do kamer RealSense bez "
                        + "hasła. Instaluje się ją raz, podając nazwę i hasło administratora."))
        return lines.joined(separator: "\n")
    }
    static func alertAction(_ action: AppAlert.Action) -> String {
        switch action {
        case .retryLiveCamera:
            return tr("Try again", "Spróbuj ponownie")
        case .useAdministratorPassword:
            return tr("Use administrator password this time", "Tym razem użyj hasła administratora")
        case .openLoginItemsSettings:
            // The Polish name of the pane ("Rzeczy otwierane podczas logowania i rozszerzenia") is too long for a
            // button; the button opens that pane of System Settings.
            return tr("Open Login Items Settings", "Otwórz Ustawienia systemowe")
        case .installCameraService(let again):
            return again ? tr("Install again", "Zainstaluj ponownie") : tr("Install", "Zainstaluj")
        }
    }

    // MARK: Camera info
    static var camera: String { tr("Camera", "Kamera") }
    static var model: String { tr("Model", "Model") }
    static var serial: String { tr("Serial number", "Numer seryjny") }
    static var firmware: String { tr("Firmware", "Firmware") }
    static var recommendedFirmware: String { tr("Recommended firmware", "Zalecany firmware") }
    static var usb: String { tr("USB", "USB") }
    static var usb2Warning: String {
        tr("USB 2 connection: use a USB 3 (USB-C) cable and port for full resolution.",
           "Połączenie USB 2: użyj kabla i portu USB 3 (USB-C), aby uzyskać pełną rozdzielczość.")
    }
    static var depthScale: String { tr("Depth unit", "Jednostka głębi") }
    static var baseline: String { tr("Baseline", "Baza stereo") }
    static var imu: String { tr("IMU", "IMU") }
    static var profiles: String { tr("Stream profiles", "Profile strumieni") }
    static var yes: String { tr("yes", "tak") }
    static var no: String { tr("no", "nie") }
    static var hardwareReset: String { tr("Hardware reset", "Reset sprzętowy") }

    // MARK: Settings
    static var settings: String { tr("Settings", "Ustawienia") }
    static var depthSection: String { tr("Depth", "Głębia") }
    static var depthResolution: String { tr("Depth resolution", "Rozdzielczość głębi") }
    static var preset: String { tr("Preset", "Preset") }
    static var highAccuracy: String { tr("High Accuracy", "Wysoka dokładność") }
    static var highDensity: String { tr("High Density", "Wysoka gęstość") }
    static var closeRange: String { tr("Close-range mode", "Tryb bliskiego zasięgu") }
    static var closeRangeHint: String {
        tr("Disparity shift: lets the face come closer (more detail) but limits the far range.",
           "Przesunięcie dysparycji: pozwala podejść bliżej (więcej szczegółów), ale ogranicza daleki zasięg.")
    }
    static var disparityShift: String { tr("Disparity shift", "Przesunięcie dysparycji") }
    static var laserPower: String { tr("Laser power", "Moc lasera") }
    static var laserMax: String { tr("Maximum", "Maksymalna") }
    static var filters: String { tr("Filters", "Filtry") }
    static var spatialFilter: String { tr("Spatial filter", "Filtr przestrzenny") }
    static var temporalFilter: String { tr("Temporal filter", "Filtr czasowy") }
    static var holeFilling: String { tr("Hole filling", "Wypełnianie dziur") }
    static var maxDistance: String { tr("Maximum distance", "Maksymalna odległość") }
    static var scanSection: String { tr("Scan", "Skan") }
    static var keepShoulders: String { tr("Keep neck and shoulders", "Zachowaj szyję i ramiona") }
    static var shouldersNeedNewScan: String {
        tr("The current scan cut off the neck and shoulders: scan again to keep them.",
           "Obecny skan odciął szyję i ramiona: zeskanuj ponownie, aby je zachować.")
    }
    static var diagonalPoses: String { tr("Diagonal poses (optional)", "Pozy ukośne (opcjonalne)") }
    static var framesPerPose: String { tr("Frames per pose", "Klatki na pozę") }
    static var scanSounds: String { tr("Scan sounds", "Dźwięki skanu") }
    static var scanSoundsHelp: String {
        tr("A tick when the head reaches a pose (hold still), a chime when the pose is captured, a fanfare when the scan is complete",
           "Krótki dźwięk po osiągnięciu pozy (nie ruszaj się), inny po jej zaliczeniu i fanfara po zakończeniu skanu")
    }
    static var reconstructionSection: String { tr("Reconstruction", "Rekonstrukcja") }
    static var voxelSize: String { tr("Voxel size", "Rozmiar woksela") }
    static var surfaceMethod: String { tr("Surface", "Powierzchnia") }
    static var methodTSDF: String { tr("TSDF (GPU)", "TSDF (GPU)") }
    static var methodPoisson: String { tr("Screened Poisson", "Screened Poisson") }
    static var closeHead: String { tr("Close the back of the head", "Domknij tył głowy") }
    static var closeHeadHelp: String {
        tr("Closes the unscanned back of the head and the neck with a smooth synthesized surface, so the model is "
           + "watertight. The closed part is a guess, not a measurement.",
           "Domyka nieskanowany tył głowy i szyję gładką, wygenerowaną powierzchnią, aby model był szczelny. "
           + "Domknięta część jest przybliżeniem, nie pomiarem.")
    }
    static var displaySection: String { tr("Display", "Wyświetlanie") }
    static var mirrored: String { tr("Mirror the preview", "Lustrzany podgląd") }
    static var landmarks: String { tr("Show face landmarks", "Pokaż punkty twarzy") }
    static var autoExposureFace: String { tr("Auto exposure on the face", "Automatyczna ekspozycja na twarz") }
    static var resetSettings: String { tr("Restore defaults", "Przywróć domyślne") }
    static var settingsLockedDuringScan: String {
        tr("Camera settings are locked during a scan.", "Ustawienia kamery są zablokowane podczas skanu.")
    }
    static var settingsLockedDuringRecording: String {
        tr("Stop the recording to change the depth settings.", "Zatrzymaj nagrywanie, aby zmienić ustawienia głębi.")
    }

    // MARK: Preview
    static var previewColor: String { tr("Color", "Kolor") }
    static var previewDepth: String { tr("Depth", "Głębia") }
    static var previewBlend: String { tr("Overlay", "Nakładka") }
    static var previewUnavailable: String { tr("Preview unavailable", "Podgląd niedostępny") }
    static var waitingForFrames: String { tr("Waiting for frames…", "Oczekiwanie na klatki…") }
    static var startScan: String { tr("Start scan", "Rozpocznij skan") }
    static var record: String { tr("Record", "Nagrywaj") }
    static var stopRecording: String { tr("Stop recording", "Zatrzymaj nagrywanie") }
    static func recordingStatus(_ seconds: Int) -> String {
        tr("Recording", "Nagrywanie") + String(format: " %d:%02d", seconds / 60, seconds % 60)
    }
    static func recordingSaved(_ bytes: Int64, _ name: String) -> String {
        let size = ByteCountFormatter.string(fromByteCount: bytes, countStyle: .file)
        return tr("Recording saved: \(name) (\(size))", "Nagranie zapisane: \(name) (\(size))")
    }
    static func fps(_ display: Double, _ analysis: Double) -> String {
        String(format: tr("%.0f fps · face %.0f fps", "%.0f kl/s · twarz %.0f kl/s"), display, analysis)
    }
    static var playbackFinished: String { tr("The recording has ended.", "Nagranie się skończyło.") }

    // MARK: Distance
    static var distance: String { tr("Distance", "Odległość") }
    static var moveCloser: String { tr("Move closer", "Podejdź bliżej") }
    static var moveBack: String { tr("Move back", "Odsuń się") }
    static var goodDistance: String { tr("Good distance", "Dobra odległość") }
    static var noFace: String { tr("No face detected", "Nie wykryto twarzy") }
    static func meters(_ m: Float) -> String { String(format: "%.2f m", m) }
    static func recommended(_ range: ClosedRange<Float>) -> String {
        tr("recommended", "zalecane") + String(format: " %.2f–%.2f m", range.lowerBound, range.upperBound)
    }
    static var estimated: String { tr("estimated, no depth", "szacunkowo, brak głębi") }

    // MARK: Scan
    static var guidedScan: String { tr("Guided scan", "Skan prowadzony") }
    static var overallProgress: String { tr("Overall", "Całość") }
    static var coverage: String { tr("Covered head regions", "Pokryte obszary głowy") }
    /// The skip button, naming the pose it skips ("Skip Up-left").
    static func skipPose(_ pose: ScanPose) -> String {
        tr("Skip \(poseName(pose))", "Pomiń: \(poseName(pose).lowercased())")
    }
    static var skipRemainingOptionalPoses: String {
        tr("Skip the remaining optional poses", "Pomiń pozostałe pozy opcjonalne")
    }
    static var finishScan: String { tr("Finish", "Zakończ") }
    static var cancelScan: String { tr("Cancel", "Anuluj") }
    static func keyframes(_ n: Int) -> String {
        tr("\(n) \(n == 1 ? "frame" : "frames")", "\(n) \(polishPlural(n, "klatka", "klatki", "klatek"))")
    }
    static var scanComplete: String { tr("Scan complete", "Skan zakończony") }
    static func scanSummary(_ frames: Int, _ poses: Int) -> String {
        tr("\(frames) \(frames == 1 ? "frame" : "frames") from \(poses) \(poses == 1 ? "pose" : "poses") captured.",
           "Zarejestrowano \(frames) \(polishPlural(frames, "klatkę", "klatki", "klatek")) z \(poses) \(poses == 1 ? "pozy" : "póz").")
    }

    /// Polish noun form for a count: 1 -> one, 2-4 (not 12-14) -> few, otherwise many.
    static func polishPlural(_ n: Int, _ one: String, _ few: String, _ many: String) -> String {
        if n == 1 { return one }
        let lastTwo = n % 100, last = n % 10
        return (2 ... 4).contains(last) && !(12 ... 14).contains(lastTwo) ? few : many
    }
    static var reconstruct: String { tr("Reconstruct 3D model", "Rekonstruuj model 3D") }
    static var reconstructAgain: String { tr("Reconstruct again", "Rekonstruuj ponownie") }
    static var resultSettingsChanged: String {
        tr("The reconstruction settings changed after this model was made.",
           "Ustawienia rekonstrukcji zmieniły się po utworzeniu tego modelu.")
    }
    static func useDatasetFrames(_ n: Int) -> String {
        tr("Use the saved frames (\(n))", "Użyj zapisanych klatek (\(n))")
    }
    static var newScan: String { tr("New scan", "Nowy skan") }
    static var backToPreview: String { tr("Back to preview", "Powrót do podglądu") }
    static var noKeyframes: String { tr("No frames were captured.", "Nie zarejestrowano żadnych klatek.") }

    static func poseName(_ pose: ScanPose) -> String {
        switch pose {
        case .front: return tr("Front", "Prosto")
        case .up: return tr("Up", "Góra")
        case .down: return tr("Down", "Dół")
        case .left: return tr("Left", "Lewo")
        case .right: return tr("Right", "Prawo")
        case .upLeft: return tr("Up-left", "Góra-lewo")
        case .upRight: return tr("Up-right", "Góra-prawo")
        case .downLeft: return tr("Down-left", "Dół-lewo")
        case .downRight: return tr("Down-right", "Dół-prawo")
        }
    }

    static func poseInstruction(_ pose: ScanPose) -> String {
        switch pose {
        case .front: return tr("Look straight at the camera", "Patrz prosto w kamerę")
        case .up: return tr("Raise your chin, look up", "Unieś brodę, spójrz w górę")
        case .down: return tr("Lower your chin, look down", "Opuść brodę, spójrz w dół")
        case .left: return tr("Turn your head to your left", "Obróć głowę w swoją lewą stronę")
        case .right: return tr("Turn your head to your right", "Obróć głowę w swoją prawą stronę")
        case .upLeft: return tr("Look up and to your left", "Spójrz w górę i w lewo")
        case .upRight: return tr("Look up and to your right", "Spójrz w górę i w prawo")
        case .downLeft: return tr("Look down and to your left", "Spójrz w dół i w lewo")
        case .downRight: return tr("Look down and to your right", "Spójrz w dół i w prawo")
        }
    }

    /// The banner text: a turn that is not within the tolerance says which way to correct it, and with no active pose
    /// the directions still open.
    static func instruction(for update: ScanUpdate) -> String {
        if case .turn = update.instructionKind, let hint = update.turnHint, hint != .further { return turnHint(hint) }
        if update.instructionKind == .turnAnyDirection { return turnAnyDirection(update.unfinishedPoses) }
        return instruction(update.instructionKind)
    }

    /// Turn toward any of the given poses (the front pose left out). Two to four are listed, the last after "or", so
    /// that no order shows: "Turn your head up, down, to your left or right" (the sides last, as one item when both
    /// are open). One gets its own instruction; more than four only "Turn your head in any direction", the pose
    /// guide's arrows showing which.
    static func turnAnyDirection(_ poses: [ScanPose]) -> String {
        let anyDirection = tr("Turn your head in any direction", "Obróć głowę w dowolną stronę")
        let open = ScanPose.allCases.filter { $0 != .front && poses.contains($0) }
        if open.count == 1 { return poseInstruction(open[0]) }
        guard (2 ... 4).contains(open.count) else { return anyDirection }
        var items = open.filter { $0 != .left && $0 != .right }.map(direction)
        let sides = open.contains(.left) && open.contains(.right)
        if sides {
            items.append(tr("to your left or right", "w swoją lewą lub prawą stronę"))
        } else if let side = open.first(where: { $0 == .left || $0 == .right }) {
            items.append(direction(side))
        }
        // The two sides already end in "or".
        let list = sides ? items.joined(separator: ", ")
            : items.dropLast().joined(separator: ", ") + tr(" or ", " lub ") + items[items.count - 1]
        return tr("Turn your head \(list)", "Obróć głowę \(list)")
    }

    /// One-letter words ("w", "z", "i" in Polish) bound to the next word with a no-break space, so that a wrapped
    /// banner does not end a line with one ("Obróć głowę w dół, w / swoją lewą lub prawą stronę").
    static func keepingShortWordsAttached(_ text: String) -> String {
        var words = text.components(separatedBy: " ")
        guard words.count > 1 else { return text }
        var result = ""
        while !words.isEmpty {
            let word = words.removeFirst()
            result += word
            guard !words.isEmpty else { break }
            result += word.count == 1 && word.first?.isLetter == true ? "\u{00A0}" : " "
        }
        return result
    }

    /// A pose as a direction to turn in (the list of turnAnyDirection); the sides person-relative, like the poses.
    static func direction(_ pose: ScanPose) -> String {
        switch pose {
        case .front: return tr("straight ahead", "prosto")
        case .up: return tr("up", "w górę")
        case .down: return tr("down", "w dół")
        case .left: return tr("to your left", "w swoją lewą stronę")
        case .right: return tr("to your right", "w swoją prawą stronę")
        case .upLeft: return tr("up-left", "w lewo do góry")
        case .upRight: return tr("up-right", "w prawo do góry")
        case .downLeft: return tr("down-left", "w lewo w dół")
        case .downRight: return tr("down-right", "w prawo w dół")
        }
    }

    static func turnHint(_ hint: TurnHint) -> String {
        switch hint {
        case .further: return tr("Turn further", "Obróć dalej")
        case .back: return tr("Too far — turn back a little", "Za daleko — cofnij głowę trochę")
        case .chinUp: return tr("Raise your chin a little", "Unieś lekko brodę")
        case .chinDown: return tr("Lower your chin a little", "Opuść lekko brodę")
        case .faceLeft: return tr("Turn your face a little to your left", "Obróć twarz lekko w swoją lewą stronę")
        case .faceRight: return tr("Turn your face a little to your right", "Obróć twarz lekko w swoją prawą stronę")
        }
    }

    static func instruction(_ instruction: ScanInstruction) -> String {
        switch instruction {
        case .idle: return ""
        case .showFace: return tr("Look at the camera", "Spójrz w kamerę")
        case .moveCloser: return moveCloser
        case .moveBack: return moveBack
        case .lookAtCamera: return tr("Hold still, looking at the camera", "Nie ruszaj się, patrz w kamerę")
        case .turn(let pose, _, _): return poseInstruction(pose)
        case .turnAnyDirection: return turnAnyDirection([])
        case .straightenHead: return tr("Keep your head straight", "Trzymaj głowę prosto")
        case .holdStill: return tr("Hold still…", "Nie ruszaj się…")
        case .capturing: return tr("Capturing — hold still", "Rejestracja — nie ruszaj się")
        case .finished: return scanComplete
        }
    }

    static func rejection(_ rejection: ScanRejection) -> String {
        switch rejection {
        case .noFace: return noFace
        case .lowConfidence: return tr("Face not clearly visible", "Twarz nie jest wyraźnie widoczna")
        case .trackingFailed: return tr("Face tracking failed", "Śledzenie twarzy nie powiodło się")
        case .tooClose: return tr("Too close", "Za blisko")
        case .tooFar: return tr("Too far", "Za daleko")
        case .noDepth: return tr("No depth on the face", "Brak głębi na twarzy")
        case .headMoving: return tr("Moving too fast", "Za szybki ruch")
        case .cameraMoving: return tr("The camera moved — keep it still", "Kamera się poruszyła — nie ruszaj jej")
        case .rollTooLarge: return tr("Head tilted sideways", "Głowa przechylona na bok")
        case .blurry: return tr("Image blurred", "Obraz rozmazany")
        case .lowDepthCoverage: return tr("Depth has holes on the face", "Dziury w głębi na twarzy")
        case .tooFewDepthPoints: return tr("Too few depth points", "Za mało punktów głębi")
        }
    }

    // MARK: Reconstruction and result
    static var reconstructing: String { tr("Reconstructing…", "Rekonstrukcja…") }
    static var cancel: String { tr("Cancel", "Anuluj") }
    static func stageName(_ stage: ReconstructionProgress.Stage) -> String {
        switch stage {
        case .preparing: return tr("Preparing", "Przygotowanie")
        case .segmentation: return tr("Segmentation", "Segmentacja")
        case .registration: return tr("Registration", "Rejestracja")
        case .globalOptimization: return tr("Global optimization", "Optymalizacja globalna")
        case .fusion: return tr("Fusion", "Fuzja")
        case .meshing: return tr("Meshing", "Tworzenie siatki")
        case .cleanup: return tr("Cleanup", "Czyszczenie")
        case .texturing: return tr("Texturing", "Teksturowanie")
        case .finished: return tr("Finished", "Gotowe")
        }
    }
    /// The pipeline's progress detail (ReconstructionProgress.message, English from ReconKit and the core) in the UI
    /// language. The Polish UI translates the details the pipeline sends today and leaves out any other: the stage
    /// name and the bar still show, and no English appears in a Polish window.
    static func progressDetail(_ message: String) -> String {
        guard AppLanguage.current == .polish else { return message }
        return polishProgressDetail(message) ?? ""
    }

    static func polishProgressDetail(_ message: String) -> String? {
        let fixed: [String: String] = [
            "head masks": "maski głowy",
            "building clouds": "budowanie chmur punktów",
            "pairwise registration": "rejestracja par klatek",
            "pose graph optimization": "optymalizacja grafu póz",
            "registration finished": "rejestracja zakończona",
            "registration finished (some frames unregistered)": "rejestracja zakończona (część klatek bez rejestracji)",
            "checking registration": "sprawdzanie rejestracji",
            "marching cubes": "marching cubes",
            "merging registered clouds": "łączenie zarejestrowanych chmur punktów",
            "poisson: solving": "Poisson: rozwiązywanie",
            "poisson: trimming": "Poisson: przycinanie",
            "poisson: finished": "Poisson: gotowe",
            "cleanup": "czyszczenie siatki",
            "texturing": "teksturowanie",
        ]
        if message.isEmpty { return "" }
        if let text = fixed[message] { return text }
        if let m = message.wholeMatch(of: #/(\d+) keyframes/#), let n = Int(m.1) {
            return "\(n) \(polishPlural(n, "klatka", "klatki", "klatek"))"
        }
        if let m = message.wholeMatch(of: #/(\d+) usable keyframes/#), let n = Int(m.1) {
            return "\(n) \(polishPlural(n, "użyteczna klatka", "użyteczne klatki", "użytecznych klatek"))"
        }
        if let m = message.wholeMatch(of: #/integrating frame (\d+) of (\d+)/#) {
            return "integracja klatki \(m.1) z \(m.2)"
        }
        if let m = message.wholeMatch(of: #/screened Poisson \(depth (\d+)\)/#) {
            return "screened Poisson (głębokość \(m.1))"
        }
        if let m = message.wholeMatch(of: #/(\d+) vertices, (\d+) triangles/#), let v = Int(m.1), let t = Int(m.2) {
            return "\(v) \(polishPlural(v, "wierzchołek", "wierzchołki", "wierzchołków")), "
                + "\(t) \(polishPlural(t, "trójkąt", "trójkąty", "trójkątów"))"
        }
        return nil
    }
    static var result: String { tr("3D model", "Model 3D") }
    static var saveOBJ: String { tr("Save OBJ…", "Zapisz OBJ…") }
    static var savePLY: String { tr("Save PLY…", "Zapisz PLY…") }
    static var saveSession: String { tr("Save raw session…", "Zapisz surową sesję…") }
    static var showInFinder: String { tr("Show in Finder", "Pokaż w Finderze") }
    static func saved(_ name: String) -> String { tr("Saved \(name)", "Zapisano \(name)") }
    static var report: String { tr("Reconstruction report", "Raport rekonstrukcji") }
    static var displayMode: String { tr("Display", "Widok") }
    static func displayModeName(_ mode: ModelDisplayMode) -> String {
        switch mode {
        case .texture: return tr("Texture", "Tekstura")
        case .vertexColors: return tr("Vertex colors", "Kolory wierzchołków")
        case .shaded: return tr("Shaded", "Cieniowanie")
        case .wireframe: return tr("Wireframe", "Siatka")
        }
    }

    // MARK: 3D preview
    /// Width × height × depth of the exported model in millimeters.
    static func modelSize(_ mm: SIMD3<Float>) -> String {
        String(format: "%.0f × %.0f × %.0f mm", mm.x, mm.y, mm.z) + tr(" (W × H × D)", " (szer. × wys. × gł.)")
    }
    static func triangles(_ n: Int) -> String {
        tr("\(n.formatted()) \(n == 1 ? "triangle" : "triangles")",
           "\(n.formatted()) \(polishPlural(n, "trójkąt", "trójkąty", "trójkątów"))")
    }
    static func textureSize(_ width: Int, _ height: Int) -> String {
        tr("texture", "tekstura") + " \(width)×\(height)"
    }
    /// Why the preview shows another mode than the one chosen (the result has no texture or no vertex colors).
    static func displayFallback(requested: ModelDisplayMode, shown: ModelDisplayMode) -> String? {
        guard requested != shown else { return nil }
        let missing: String
        switch requested {
        case .texture: missing = tr("No texture", "Brak tekstury")
        case .vertexColors: missing = tr("No vertex colors", "Brak kolorów wierzchołków")
        case .shaded, .wireframe: return nil
        }
        let showing: String
        switch shown {
        case .vertexColors: showing = tr("showing vertex colors", "pokazano kolory wierzchołków")
        default: showing = tr("showing shading", "pokazano cieniowanie")
        }
        return missing + ": " + showing
    }
    static var resetView: String { tr("Reset the view", "Przywróć widok") }
    static var reportKeyframes: String { tr("Keyframes (used / total)", "Klatki (użyte / wszystkie)") }
    static var reportRMSE: String { tr("Registration RMSE (mean / max)", "RMSE rejestracji (średnie / maks.)") }
    static var reportFitness: String { tr("Mean fitness", "Średnie dopasowanie") }
    static var reportEdges: String { tr("Odometry / loop closures / rejected", "Odometria / pętle / odrzucone") }
    static var reportVoxel: String { tr("Voxel size", "Rozmiar woksela") }
    static var reportMethod: String { tr("Method", "Metoda") }
    static var reportMesh: String { tr("Vertices / triangles", "Wierzchołki / trójkąty") }
    static var reportTimings: String { tr("Timings", "Czasy") }
    /// Stage names of ReconstructionReport.durations.
    static func timingName(_ key: String) -> String {
        switch key {
        case "undistortion": return tr("Lens undistortion", "Usuwanie dystorsji obiektywu")
        case "segmentation": return stageName(.segmentation)
        case "registration": return stageName(.registration)
        case "registration check": return tr("Registration check", "Kontrola rejestracji")
        case "bounds": return tr("Volume bounds", "Granice objętości")
        case "fusion": return stageName(.fusion)
        case "poisson": return tr("Poisson surface", "Powierzchnia Poissona")
        case "cleanup": return stageName(.cleanup)
        case "texturing": return stageName(.texturing)
        default: return key
        }
    }
    static var reportNotes: String { tr("Notes", "Uwagi") }
    /// A note of ReconstructionReport.notes (English from ReconstructionPipeline) in the UI language. The Polish UI
    /// translates the notes the pipeline writes today; an unknown note stays in English rather than disappearing
    /// (unlike a progress detail, it is the only place its information shows). Error details stay as they are.
    static func reportNote(_ note: String) -> String {
        guard AppLanguage.current == .polish else { return note }
        return polishReportNote(note) ?? note
    }

    static func polishReportNote(_ note: String) -> String? {
        let fixed: [String: String] = [
            "no depth at the chin: neck not cropped": "brak głębi przy brodzie: szyja nie została przycięta",
            "Metal unavailable: fused on the CPU": "Metal niedostępny: fuzja na CPU",
        ]
        if let text = fixed[note] { return text }
        // Extended syntax (multi-line literal): whitespace is ignored, "\ " is a space.
        let initialPoses = #/
            initial\ poses:\ (\d+)\ landmark\ fits,\ (\d+)\ Vision\ rotation\ \+\ landmark\ translation,
            \ (-?\d+)\ Vision\ rotation\ \+\ head\ centroid
            /#
        if let m = note.wholeMatch(of: initialPoses) {
            return "pozy początkowe: \(m.1) z dopasowania punktów twarzy, \(m.2) z obrotu Vision i przesunięcia z punktów "
                + "twarzy, \(m.3) z obrotu Vision i środka głowy"
        }
        if let m = note.wholeMatch(of: #/keyframes excluded after registration \((surface|face landmarks) do(?:es)? not fit\): (.+)/#) {
            let reason = m.1 == "surface" ? "powierzchnia nie pasuje" : "punkty twarzy nie pasują"
            return "klatki wykluczone po rejestracji (\(reason)): \(m.2)"
        }
        if let m = note.wholeMatch(of: #/(\d+) of (\d+) cm² synthesized by hole filling: not measured/#) {
            return "\(m.1) z \(m.2) cm² wygenerowano przy wypełnianiu dziur: to nie pomiar"
        }
        if let m = note.wholeMatch(of: #/lens distortion removed: (.+)/#) {
            return "usunięto dystorsję obiektywu: \(m.1)"
        }
        if let m = note.wholeMatch(of: #/unknown lens distortion model (.+): treated as a pinhole/#) {
            return "nieznany model dystorsji obiektywu \(m.1): przyjęto model otworkowy"
        }
        if let m = note.wholeMatch(of: #/(\d+) keyframes with inconsistent image data skipped/#), let n = Int(m.1) {
            return "pominięto \(n) \(polishPlural(n, "klatkę", "klatki", "klatek")) z niespójnymi danymi obrazu"
        }
        if let m = note.wholeMatch(of: #/(\d+) keyframes without usable head depth skipped/#), let n = Int(m.1) {
            return "pominięto \(n) \(polishPlural(n, "klatkę", "klatki", "klatek")) bez użytecznej głębi głowy"
        }
        if let m = note.wholeMatch(of: #/volume too large for ([\d.]+) mm voxels; using ([\d.]+) mm/#) {
            return "objętość za duża dla wokseli \(m.1) mm; użyto \(m.2) mm"
        }
        if let m = note.wholeMatch(of: #/chin crop: triangle-level fallback \((.*)\)/#) {
            return "przycięcie pod brodą: zastępczo całymi trójkątami (\(m.1))"
        }
        if let m = note.wholeMatch(of: #/(mesh cleanup|vertex colors|texture) failed: (.*)/#) {
            let step = ["mesh cleanup": "czyszczenie siatki", "vertex colors": "kolory wierzchołków",
                        "texture": "tekstura"][String(m.1)] ?? String(m.1)
            return "błąd kroku „\(step)”: \(m.2)"
        }
        return nil
    }
    static var gpu: String { tr("GPU", "GPU") }
    static var cpu: String { tr("CPU", "CPU") }

    // MARK: Errors
    static var error: String { tr("Error", "Błąd") }
    static var ok: String { tr("OK", "OK") }
    static var couldNotOpenSource: String { tr("Could not open the source", "Nie można otworzyć źródła") }
    static var couldNotConfigure: String { tr("Could not apply the camera settings", "Nie można zastosować ustawień kamery") }
    static var cameraError: String { tr("Camera error", "Błąd kamery") }
    static var cameraDisconnected: String { tr("The camera was disconnected", "Kamera została odłączona") }
    static var helperExited: String { tr("The camera helper stopped", "Proces pomocniczy kamery zakończył działanie") }
    static func exitStatus(_ status: Int32) -> String { tr("Exit status \(status)", "Kod wyjścia \(status)") }
    /// The pipeline's own errors in the UI language; the core's detail message stays as it is (English).
    static func reconstructionError(_ error: Error) -> String {
        switch error as? ReconstructionError {
        case .notEnoughKeyframes(let n)?:
            return tr("Not enough usable frames (\(n)). Scan again with the face well lit and at the recommended distance.",
                      "Za mało użytecznych klatek (\(n)). Zeskanuj ponownie z dobrze oświetloną twarzą w zalecanej odległości.")
        case .core(let stage, _, let message)?:
            return tr("Stage “\(stage)” failed: \(message)", "Etap „\(stage)” nie powiódł się: \(message)")
        case .cancelled?:
            return tr("The reconstruction was cancelled.", "Rekonstrukcja została anulowana.")
        case nil:
            return error.localizedDescription
        }
    }
    static var noMetalDevice: String { tr("No Metal device.", "Brak urządzenia Metal.") }
    static var noMetalQueue: String { tr("No Metal command queue.", "Brak kolejki poleceń Metal.") }
    static func previewShaders(_ detail: String) -> String {
        tr("Preview shaders: ", "Shadery podglądu: ") + detail
    }
    static var recordingFailed: String { tr("Recording failed", "Nagrywanie nie powiodło się") }
    static var reconstructionFailed: String { tr("Reconstruction failed", "Rekonstrukcja nie powiodła się") }
    static var scanFailed: String {
        tr("The scan stopped: face tracking keeps failing", "Skan przerwany: śledzenie twarzy stale zawodzi")
    }
    static var saveFailed: String { tr("Could not save", "Nie można zapisać") }
    static var quitting: String { tr("Releasing the camera…", "Zwalnianie kamery…") }
}
