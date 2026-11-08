import QtQuick
import QtTest
import "../../qml" as UI

Item {
    width: 1460; height: 940
    UI.Main { id: app; visible: true }
    TestCase {
        parent: app.contentItem
        name: "ConfigurationWorkflow"
        when: windowShown
        function initTestCase() {
            studio.openDemo()
            app.requestActivate()
            tryCompare(studio, "connected", true)
            tryCompare(studio, "pendingCount", 0)
            tryCompare(studio, "busy", false)
        }
        function button(name) {
            let item = findChild(app.contentItem, name)
            verify(item !== null, "Missing button: " + name)
            tryCompare(item, "enabled", true)
            return item
        }
        function status(label) {
            for (let i = 0; i < studio.motorStatus.length; ++i)
                if (studio.motorStatus[i].label === label) return studio.motorStatus[i].value
            return ""
        }
        function test_a_sevenIndependentPages() {
            let counts = [26, 11, 17, 8, 58, 36, 10]
            for (let i = 0; i < 7; ++i) {
                mouseClick(button("page-" + i))
                compare(app.selectedPage, i)
                let list = findChild(app.contentItem, "parameterList")
                tryCompare(list, "count", counts[i])
            }
        }
        function test_b_backgroundPollKeepsEditorFocus() {
            mouseClick(button("page-0"))
            tryCompare(studio, "busy", false)
            tryVerify(function(){return findChild(app.contentItem, "editor-motor0_rated_voltage") !== null})
            let editor = findChild(app.contentItem, "editor-motor0_rated_voltage")
            app.requestActivate()
            tryCompare(app, "active", true)
            // Let the page transition finish polishing before establishing the
            // focus whose survival across background polling is being tested.
            wait(60)
            editor = findChild(app.contentItem, "editor-motor0_rated_voltage")
            verify(editor !== null)
            let focusEvents = []
            let focusTrace = function(){focusEvents.push("focus=" + editor.activeFocus + ", enabled=" + editor.enabled + ", busy=" + studio.busy + ", activeWindow=" + app.active)}
            editor.activeFocusChanged.connect(focusTrace)
            editor.forceActiveFocus()
            tryCompare(editor, "activeFocus", true)
            wait(1200)
            editor.activeFocusChanged.disconnect(focusTrace)
            verify(editor.activeFocus, "Background polling focus trace: " + focusEvents.join("; "))
        }
        function test_b_editWriteReadbackApplyAndControl() {
            mouseClick(button("page-0"))
            tryCompare(studio, "busy", false)
            tryVerify(function(){return findChild(app.contentItem, "editor-motor0_rated_voltage") !== null})
            let editor = findChild(app.contentItem, "editor-motor0_rated_voltage")
            verify(editor !== null)
            editor.forceActiveFocus()
            app.requestActivate()
            tryCompare(editor, "activeFocus", true)
            keyClick(Qt.Key_A, Qt.ControlModifier)
            keyClick(Qt.Key_1); keyClick(Qt.Key_2); keyClick(Qt.Key_Period); keyClick(Qt.Key_5)
            keyClick(Qt.Key_Return)
            tryCompare(studio.dictionary, "dirtyCount", 1)
            mouseClick(button("writeGroup"))
            tryCompare(studio.dictionary, "dirtyCount", 0)
            let device = findChild(app.contentItem, "device-motor0_rated_voltage")
            tryCompare(device, "text", "12.5")
            tryVerify(function(){return studio.configState.indexOf("需要应用") >= 0})
            mouseClick(button("applyConfig"))
            tryVerify(function(){return studio.configState.indexOf("无待应用") >= 0})
            mouseClick(button("page-7"))
            mouseClick(button("motorEnable"))
            tryVerify(function(){return status("使能") === "1"})
            mouseClick(button("motorDisable"))
            tryVerify(function(){return status("使能") === "0"})
            mouseClick(button("page-0"))
            mouseClick(button("saveFlash"))
            tryVerify(function(){return studio.message.indexOf("Flash 保存成功") >= 0})
        }
        function test_c_disconnectPreservesUnsentDraft() {
            tryCompare(studio, "busy", false)
            mouseClick(button("page-0"))
            tryVerify(function(){return findChild(app.contentItem, "editor-motor0_rated_voltage") !== null})
            let editor = findChild(app.contentItem, "editor-motor0_rated_voltage")
            editor.forceActiveFocus()
            app.requestActivate()
            tryCompare(editor, "activeFocus", true)
            keyClick(Qt.Key_A, Qt.ControlModifier)
            keyClick(Qt.Key_1); keyClick(Qt.Key_4); keyClick(Qt.Key_Return)
            tryCompare(studio.dictionary, "dirtyCount", 1)
            studio.disconnectPort()
            tryCompare(studio, "connected", false)
            compare(studio.dictionary.dirtyCount, 1)
            compare(editor.text, "14")
            let device = findChild(app.contentItem, "device-motor0_rated_voltage")
            compare(device.text, "—")
        }
        function test_d_scopeContinuousZoomCursorsAndStop() {
            studio.openDemo()
            tryCompare(studio, "connected", true)
            tryCompare(studio, "busy", false)
            mouseClick(button("page-8"))
            studio.scope.mode = 1
            mouseClick(button("scopeStart"))
            tryCompare(studio.scope, "active", true)
            tryVerify(function(){return (studio.scope.info.received || 0) > 2500})
            let plot = findChild(app.contentItem, "scopePlot")
            tryVerify(function(){return plot.displayPoints > 0})
            let interaction = findChild(app.contentItem, "scopeInteraction")
            mouseClick(interaction, interaction.width * 0.6, interaction.height * 0.5, Qt.LeftButton)
            mouseClick(interaction, interaction.width * 0.8, interaction.height * 0.5, Qt.RightButton)
            compare(studio.scope.following, false)
            verify(studio.scope.cursorText.indexOf("CH1=") >= 0)
            verify(studio.scope.cursorText.indexOf("Δt") >= 0)
            let previousSpan = studio.scope.span
            mouseClick(button("scopeZoomIn"))
            verify(studio.scope.span < previousSpan)
            mouseClick(button("scopeStop"))
            tryCompare(studio.scope, "active", false)
            compare(studio.scope.info.error, "")
        }
        function test_e_scopeSingleBurst() {
            studio.scope.rate = 10000
            studio.scope.mode = 2
            mouseClick(button("scopeSingle"))
            compare(studio.scope.singleShot, true)
            mouseClick(button("scopeStart"))
            tryCompare(studio.scope, "active", true)
            tryCompare(studio.scope, "active", false)
            compare(studio.scope.info.received, 1360)
            compare(studio.scope.info.segments, 1)
            compare(studio.scope.info.error, "")
        }
        function test_f_recordPlaybackAndExport() {
            studio.scope.singleShot = false
            studio.scope.recordUrl = recordingTestUrl
            mouseClick(button("scopeRecord"))
            verify(studio.scope.recordEnabled)
            mouseClick(button("scopeStart"))
            tryCompare(studio.scope, "active", true)
            tryVerify(function(){return (studio.scope.info.segments || 0) >= 3})
            mouseClick(button("scopeStop"))
            tryCompare(studio.scope, "active", false)
            tryCompare(studio.scope, "fileBusy", false)
            verify(studio.scope.info.recordComplete, JSON.stringify(studio.scope.info))
            let samples = studio.scope.info.received
            studio.scope.openRecording(recordingTestUrl)
            tryCompare(studio.scope, "fileBusy", false)
            compare(studio.scope.state, "文件回放")
            compare(studio.scope.info.received, samples)
            verify(findChild(app.contentItem, "scopePlot").displayPoints > 0)
            let position = findChild(app.contentItem, "scopeReplayPosition")
            tryVerify(function(){return Math.abs(position.value - studio.scope.viewEnd) < 0.001}, 5000,
                      "Replay slider must follow the visible device time")
            let target = studio.scope.info.replayStart + 0.02
            studio.scope.seekRecording(target)
            tryCompare(studio.scope, "fileBusy", false)
            compare(studio.scope.state, "文件回放")
            fuzzyCompare(studio.scope.viewEnd, target, 0.00001)
            studio.scope.exportRecording(recordingTestUrl, exportTestUrl)
            tryCompare(studio.scope, "fileBusy", false)
            verify(studio.scope.fileMessage.indexOf("CSV 导出完成") >= 0, studio.scope.fileMessage)
            studio.scope.openRecording(exportTestUrl)
            tryCompare(studio.scope, "fileBusy", false)
            verify(studio.scope.fileMessage.indexOf("失败") >= 0)
            compare(studio.scope.info.received, samples)
        }
        function test_g_parameterSchemeAndMotion() {
            studio.scope.recordEnabled = false
            mouseClick(button("page-0"))
            mouseClick(button("readAll"))
            tryCompare(studio, "busy", false)
            verify(studio.saveScheme(schemeTestUrl))
            verify(studio.previewScheme(schemeTestUrl))
            verify(studio.applyScheme())
            wait(40)
            verify(uiHarness.saveWindow(app, "phase5-config.png"))
            mouseClick(button("page-7"))
            mouseClick(button("motorEnable"))
            tryVerify(function(){return status("使能") === "1"})
            studio.command("align")
            tryCompare(studio, "busy", false)
            mouseClick(button("motionSubmit"))
            tryVerify(function(){return !!studio.motion.active})
            wait(40)
            verify(uiHarness.saveWindow(app, "phase5-motor.png"))
            mouseClick(button("motionStop"))
            tryVerify(function(){return !studio.motion.active})
            studio.refreshStatus()
            tryVerify(function(){return status("使能") === "0"})
        }
        function test_h_fftAndSeparateAxes() {
            mouseClick(button("page-8"))
            studio.scope.openRecording(recordingTestUrl)
            tryCompare(studio.scope, "fileBusy", false)
            mouseClick(button("scopeSeparateAxes"))
            compare(findChild(app.contentItem, "scopeSeparateAxes").checked, true)
            mouseClick(button("scopeAnalysisOpen"))
            let dialog = findChild(app.contentItem, "scopePage").analysisPopup
            verify(dialog !== null, "Missing analysis dialog")
            let analyzeButton = findChild(dialog.contentItem, "scopeAnalyze")
            verify(analyzeButton !== null)
            mouseClick(analyzeButton)
            tryCompare(studio.scope, "analysisBusy", false)
            verify((studio.scope.analysis.count || 0) >= 32, JSON.stringify(studio.scope.analysis))
            verify(studio.scope.analysis.points.length > 0)
            wait(80)
            verify(uiHarness.saveWindow(app, "phase5-fft.png"))
            dialog.close()
            wait(40)
            verify(uiHarness.saveWindow(app, "phase5-axes.png"))
        }
        function cleanupTestCase() { app.close() }
    }
}
