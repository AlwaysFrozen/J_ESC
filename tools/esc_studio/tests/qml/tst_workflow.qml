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
            mouseClick(button("page-7"))
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
            mouseClick(button("scopeFiles"))
            // Expanding file controls reflows the toolbar before the next click.
            wait(60)
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
            mouseClick(button("page-7"))
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
        function test_i_controlWhileCapturing() {
            let files = findChild(app.contentItem, "scopeFiles")
            if (files.checked) mouseClick(files)
            mouseClick(button("page-7"))
            studio.scope.recordEnabled = false
            studio.scope.singleShot = false
            studio.scope.mode = 1
            let axes = findChild(app.contentItem, "scopeSeparateAxes")
            if (axes.checked) mouseClick(axes)
            mouseClick(button("scopeStart"))
            tryCompare(studio.scope, "active", true)
            tryVerify(function(){return (studio.scope.info.received || 0) > 2500})
            compare(studio.busy, true)
            tryCompare(studio, "controlBusy", false)
            mouseClick(button("motorEnable"))
            tryVerify(function(){return status("使能") === "1"})
            studio.command("align")
            tryCompare(studio, "controlBusy", false)
            mouseClick(button("motionSubmit"))
            tryVerify(function(){return !!studio.motion.active})
            let id = studio.motion.id
            let received = studio.scope.info.received
            // Exercise heartbeat renewal concurrently with the live stream.
            wait(1200)
            verify(studio.motion.active)
            verify(studio.scope.info.received > received)
            mouseClick(button("motionSubmit"))
            tryVerify(function(){return studio.motion.id > id})
            verify(uiHarness.saveWindow(app, "control-scope-live.png"))
            mouseClick(button("motionStop"))
            tryVerify(function(){return !studio.motion.active})
            studio.refreshStatus()
            tryVerify(function(){return status("使能") === "0"})
            compare(studio.scope.active, true)
            mouseClick(button("motorEnable"))
            tryVerify(function(){return status("使能") === "1"})
            mouseClick(button("motorDisable"))
            tryVerify(function(){return status("使能") === "0"})
            compare(studio.scope.active, true)
            app.width = 1120; app.height = 740
            wait(100)
            let plot = findChild(app.contentItem, "scopePlot")
            let stop = button("motorDisable")
            let start = button("motionSubmit")
            verify(plot.visible && plot.width > 400 && plot.height >= 120)
            let page = findChild(app.contentItem, "scopePage")
            verify(plot.mapToItem(page, 0, plot.height).y <= page.height, "Plot must fit inside the page")
            verify(stop.mapToItem(app.contentItem, 0, 0).y >= 0)
            verify(start.mapToItem(app.contentItem, 0, start.height).y < app.height)
            verify(uiHarness.saveWindow(app, "control-scope-compact.png"))
            mouseClick(button("scopeFiles"))
            wait(60)
            verify(plot.mapToItem(page, 0, plot.height).y <= page.height, "Expanded file controls must fit")
            mouseClick(button("scopeFiles"))
            wait(60)
            mouseClick(button("scopeStop"))
            tryCompare(studio.scope, "active", false)
            compare(studio.scope.info.error, "")
            app.width = 1460; app.height = 940
        }
        function dragPlot(item, x1, y1, x2, y2, which) {
            mouseMove(item, x1, y1)
            mousePress(item, x1, y1, which)
            for (let step = 1; step <= 8; ++step)
                mouseMove(item, x1 + (x2-x1)*step/8, y1 + (y2-y1)*step/8, 10)
            mouseRelease(item, x2, y2, which)
        }
        function test_j_crosshairAndDrag() {
            mouseClick(button("page-7"))
            studio.scope.openRecording(recordingTestUrl)
            tryCompare(studio.scope, "fileBusy", false)
            mouseClick(button("scopeResetView"))
            studio.scope.zoom(0.25)
            let area = findChild(app.contentItem, "scopeInteraction")
            let plot = findChild(app.contentItem, "scopePlot")
            tryVerify(function(){return plot.displayPoints > 0})
            compare(findChild(app.contentItem, "scopeCrosshair").checked, false)
            mouseMove(area, area.width*0.4, area.height*0.4)
            compare(area.crosshairVisible, false)
            mouseClick(button("scopeCrosshair"))
            mouseMove(area, area.width*0.4, area.height*0.4)
            tryCompare(area, "crosshairVisible", true)
            verify(Math.abs(area.hoverTime - (studio.scope.viewStart + studio.scope.span*0.4)) < studio.scope.span*0.005)
            verify(Math.abs(area.hoverValue - (plot.maximum - 0.4*(plot.maximum-plot.minimum))) < (plot.maximum-plot.minimum)*0.005)
            mouseClick(area, area.width*0.3, area.height*0.5, Qt.LeftButton)
            mouseClick(area, area.width*0.7, area.height*0.5, Qt.RightButton)
            let a = studio.scope.viewStart + studio.scope.cursorA*studio.scope.span
            let b = studio.scope.viewStart + studio.scope.cursorB*studio.scope.span
            let end = studio.scope.viewEnd, span = studio.scope.span
            dragPlot(area, area.width*0.4, area.height*0.5, area.width*0.6, area.height*0.5, Qt.LeftButton)
            verify(studio.scope.viewEnd < end)
            compare(studio.scope.span, span)
            compare(studio.scope.following, false)
            verify(Math.abs(studio.scope.viewStart + studio.scope.cursorA*span - a) < 1e-9)
            verify(Math.abs(studio.scope.viewStart + studio.scope.cursorB*span - b) < 1e-9)
            let low = plot.minimum, high = plot.maximum
            dragPlot(area, area.width*0.5, area.height*0.4, area.width*0.5, area.height*0.6, Qt.LeftButton)
            compare(plot.autoScale, false)
            tryVerify(function(){return plot.minimum > low})
            verify(Math.abs((plot.maximum-plot.minimum) - (high-low)) < 1e-6)
            mouseMove(area, area.width*0.55, area.height*0.45)
            verify(uiHarness.saveWindow(app, "scope-crosshair-drag.png"))
            mouseClick(button("scopeCrosshair"))
            mouseMove(area, area.width*0.4, area.height*0.4)
            compare(area.crosshairVisible, false)
            end = studio.scope.viewEnd
            dragPlot(area, area.width*0.4, area.height*0.5, area.width*0.5, area.height*0.5, Qt.MiddleButton)
            verify(studio.scope.viewEnd < end)
            mouseClick(button("scopeResetView"))
            compare(plot.autoScale, true)
            mouseClick(button("scopeSeparateAxes"))
            studio.scope.zoom(0.25)
            mouseClick(button("scopeCrosshair"))
            let channel = findChild(app.contentItem, "scopeChannelInteraction-0")
            verify(channel !== null)
            wait(60)
            mouseMove(channel, channel.width*0.5, channel.height*0.5)
            tryCompare(channel, "crosshairVisible", true)
            end = studio.scope.viewEnd
            dragPlot(channel, channel.width*0.4, channel.height*0.4, channel.width*0.6, channel.height*0.6, Qt.LeftButton)
            verify(studio.scope.viewEnd < end)
            compare(channel.waveform.autoScale, false)
            verify(uiHarness.saveWindow(app, "scope-channel-crosshair.png"))
            mouseClick(button("scopeResetView"))
            compare(channel.waveform.autoScale, true)
        }
        function cleanupTestCase() { app.close() }
    }
}
