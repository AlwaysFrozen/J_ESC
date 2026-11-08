import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import Jesc.Scope 1.0

ColumnLayout {
    id: page
    objectName: "scopePage"
    property alias analysisPopup: analysisDialog
    readonly property var capture: studio.scope
    spacing: 10
    property url exportSource
    property bool separateAxes: false
    Dialog {
        id: analysisDialog; objectName: "scopeAnalysisDialog"; title: "FFT 与原始样本统计"; width: 1000; height: 620; modal: false; anchors.centerIn: parent
        contentItem: ColumnLayout {
            RowLayout {
                ComboBox { id: analysisChannel; model: page.capture.legend; textRole: "name"; Layout.preferredWidth: 480; enabled: !page.capture.analysisBusy }
                Button { objectName: "scopeAnalyze"; text: page.capture.analysisBusy ? "正在分析" : "分析当前时间窗口"; enabled: !page.capture.analysisBusy; onClicked: page.capture.analyze(analysisChannel.currentIndex) }
            }
            Label { text: page.capture.analysis.error || "最近连续段，最多 65,536 点；去均值 + Hann 窗，单边幅值。"; color: "#d8c394"; Layout.fillWidth: true; wrapMode: Text.Wrap }
            Label {
                Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#bcd7e7"
                text: page.capture.analysis.count ? ("N=" + page.capture.analysis.count + " · 均值 " + page.capture.analysis.mean.toPrecision(6) + " · RMS " + page.capture.analysis.rms.toPrecision(6) +
                    " · 最小/最大 " + page.capture.analysis.minimum.toPrecision(6) + " / " + page.capture.analysis.maximum.toPrecision(6) + " · 峰峰值 " + page.capture.analysis.peakToPeak.toPrecision(6) + "\n主峰 " + page.capture.analysis.peakHz.toPrecision(6) +
                    " Hz · 幅值 " + page.capture.analysis.peakAmplitude.toPrecision(6) + " · 频率分辨率 " + page.capture.analysis.resolutionHz.toPrecision(5) + " Hz") : "尚未分析"
            }
            Rectangle {
                Layout.fillWidth: true; Layout.fillHeight: true; color: "#0c1724"; border.color: "#2b4257"
                Waveform { anchors.fill: parent; anchors.margins: 1; points: page.capture.analysis.points || [] }
            }
            Label { text: "频率轴：0 ～ " + (page.capture.analysis.nyquistHz || 0) + " Hz；幅值与所选通道原始单位一致。"; color: "#8ba8c0" }
        }
    }
    FileDialog { id: recordDialog; title: "选择新的原始录制文件"; fileMode: FileDialog.SaveFile; nameFilters: ["J_ESC 原始记录 (*.jescscope)"]; defaultSuffix: "jescscope"; onAccepted: page.capture.recordUrl = selectedFile }
    FileDialog { id: replayDialog; title: "打开并校验原始记录"; nameFilters: ["J_ESC 原始记录 (*.jescscope)"]; onAccepted: page.capture.openRecording(selectedFile) }
    FileDialog { id: exportSourceDialog; title: "选择要导出的原始记录"; nameFilters: ["J_ESC 原始记录 (*.jescscope)"]; onAccepted: { page.exportSource = selectedFile; csvDialog.open() } }
    FileDialog { id: csvDialog; title: "导出到新的 CSV 文件"; fileMode: FileDialog.SaveFile; nameFilters: ["CSV (*.csv)"]; defaultSuffix: "csv"; onAccepted: page.capture.exportRecording(page.exportSource, selectedFile) }
    RowLayout {
        Layout.fillWidth: true; spacing: 10
        ComboBox {
            objectName: "scopeRate"; model: ["10 kHz", "20 kHz"]; Layout.preferredWidth: 115
            currentIndex: page.capture.rate === 10000 ? 0 : 1; enabled: !page.capture.active
            onActivated: page.capture.rate = currentIndex === 0 ? 10000 : 20000
        }
        ComboBox {
            objectName: "scopeMode"; model: ["自动模式", "强制连续", "间断采样"]; Layout.preferredWidth: 135
            currentIndex: page.capture.mode; enabled: !page.capture.active
            onActivated: page.capture.mode = currentIndex
        }
        CheckBox { objectName: "scopeSingle"; text: "单段"; checked: page.capture.singleShot; enabled: !page.capture.active; onToggled: page.capture.singleShot = checked }
        Button { objectName: "scopeStart"; text: "开始采集"; enabled: studio.connected && !studio.busy && !page.capture.active && !page.capture.fileBusy; onClicked: page.capture.start() }
        Button { objectName: "scopeStop"; text: "停止 / 排空"; enabled: page.capture.active; onClicked: page.capture.stop() }
        Button { objectName: "scopeAnalysisOpen"; text: "FFT / 统计"; onClicked: analysisDialog.open() }
        Item { Layout.fillWidth: true }
        Label { text: page.capture.state; color: "#74dec0"; font.pixelSize: 12 }
    }
    RowLayout {
        Layout.fillWidth: true
        CheckBox { objectName: "scopeRecord"; text: "采集时录制"; checked: page.capture.recordEnabled; enabled: !page.capture.active && !page.capture.fileBusy; onToggled: page.capture.recordEnabled = checked }
        Button { text: "选择录制文件"; enabled: !page.capture.active && !page.capture.fileBusy; onClicked: recordDialog.open() }
        Label { text: page.capture.recordUrl.toString() || "尚未选择文件"; Layout.fillWidth: true; elide: Text.ElideMiddle; color: "#829bb7"; font.pixelSize: 10 }
        Button { objectName: "scopeReplay"; text: "打开回放"; enabled: !page.capture.active && !page.capture.fileBusy; onClicked: replayDialog.open() }
        Button { objectName: "scopeExport"; text: "导出 CSV"; enabled: !page.capture.active && !page.capture.fileBusy; onClicked: exportSourceDialog.open() }
    }
    Label {
        Layout.fillWidth: true; wrapMode: Text.Wrap; font.pixelSize: 10; color: "#96b6ce"
        text: page.capture.fileMessage || page.capture.info.recordError ||
              (page.capture.info.recordPath ? ((page.capture.info.recordBusy ? "录制 / 写盘中" : (page.capture.info.recordComplete ? "录制写盘完成" : "录制未完成")) +
              " · " + ((page.capture.info.recordBytes || 0) / 1048576).toFixed(2) + " MiB · 队列 " + ((page.capture.info.recordQueued || 0) / 1024).toFixed(0) + " KiB") : "")
        visible: text.length > 0
    }
    RowLayout {
        Layout.fillWidth: true; visible: page.capture.state === "文件回放"
        Label { text: "回放位置"; color: "#829bb7"; font.pixelSize: 10 }
        Slider {
            objectName: "scopeReplayPosition"; Layout.fillWidth: true
            from: page.capture.info.replayStart || 0; to: Math.max(from + 0.0001, page.capture.info.replayEnd || 0)
            value: Math.max(from, Math.min(to, page.capture.viewEnd)); enabled: !page.capture.fileBusy
            onPressedChanged: if(!pressed) page.capture.seekRecording(value)
        }
    }
    Label {
        visible: text.length > 0; text: page.capture.info.error || ""
        Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#f1a18e"; font.pixelSize: 11
    }
    RowLayout {
        Layout.fillWidth: true; Layout.fillHeight: true; spacing: 14
        Rectangle {
            Layout.preferredWidth: 245; Layout.fillHeight: true; color: "#142131"; radius: 6; border.color: "#26374a"
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 12; spacing: 8
                Label { text: (page.capture.state === "文件回放" ? "下次采集变量  " : "采样变量  ") + page.capture.selectedNames.length + " / 16"; color: "#c7dbea"; font.pixelSize: 12 }
                TextField { Layout.fillWidth: true; placeholderText: "搜索变量"; placeholderTextColor: "#7891ad"; onTextChanged: page.capture.search = text }
                ListView {
                    Layout.fillWidth: true; Layout.fillHeight: true; clip: true
                    model: page.capture.choices
                    ScrollBar.vertical: ScrollBar {}
                    delegate: CheckBox {
                        required property var modelData
                        width: ListView.view.width; height: 52
                        text: modelData.label
                        checked: page.capture.selectedNames.indexOf(modelData.name) >= 0
                        enabled: !page.capture.active
                        onClicked: page.capture.selectChannel(modelData.name, checked)
                        contentItem: Column {
                            leftPadding: 28; spacing: 3
                            Label { width: parent.width - 30; text: modelData.label; elide: Text.ElideRight; font.pixelSize: 10; color: "#c2d2e5" }
                            Label { width: parent.width - 30; text: modelData.name; elide: Text.ElideRight; font.pixelSize: 8; color: "#7e97b1" }
                        }
                        ToolTip.visible: hovered; ToolTip.text: modelData.name
                    }
                }
                Label { text: "仅采样只读运行变量。\n选择通道不会启动电机。"; Layout.fillWidth: true; wrapMode: Text.Wrap; font.pixelSize: 10; color: "#829bb7" }
            }
        }
        ColumnLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 8
            RowLayout {
                Button { objectName: "scopeZoomIn"; text: "＋"; Layout.preferredWidth: 40; onClicked: page.capture.zoom(0.5) }
                Button { text: "－"; Layout.preferredWidth: 40; onClicked: page.capture.zoom(2) }
                Label { text: (page.capture.span * 1000).toPrecision(4) + " ms / 窗口"; color: "#98aec6"; font.pixelSize: 10 }
                Item { Layout.fillWidth: true }
                CheckBox { objectName: "scopeFollow"; text: "跟随"; checked: page.capture.following; onToggled: page.capture.following = checked }
                CheckBox { text: "自动 Y"; enabled: !page.separateAxes; checked: page.separateAxes || plot.autoScale; onToggled: if(!page.separateAxes) plot.autoScale = checked }
                CheckBox { objectName: "scopeSeparateAxes"; text: "分通道坐标"; checked: page.separateAxes; onToggled: page.separateAxes = checked }
                Button { text: "复位视图"; onClicked: { page.capture.resetView(); plot.autoScale = true } }
            }
            Flow {
                Layout.fillWidth: true; Layout.preferredHeight: childrenRect.height; spacing: 8
                Repeater {
                    model: page.capture.legend
                    delegate: Label {
                        required property var modelData
                        width: Math.min(implicitWidth, 330); height: 20; text: "● " + modelData.name
                        color: modelData.color; font.pixelSize: 10; elide: Text.ElideRight
                    }
                }
            }
            Rectangle {
                id: plotFrame
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 200
                radius: 5; color: "#0c1724"; border.color: "#2b4257"; clip: true
                Waveform { id: plot; objectName: "scopePlot"; visible: !page.separateAxes; anchors.fill: parent; anchors.margins: 1; source: page.capture }
                ScrollView {
                    id: axesScroll
                    visible: page.separateAxes; anchors.fill: parent; clip: true
                    contentWidth: availableWidth
                    Column {
                        id: axesColumn; width: axesScroll.availableWidth; spacing: 4
                        Repeater {
                            model: page.capture.legend
                            delegate: Rectangle {
                                required property int index
                                required property var modelData
                                width: axesColumn.width; height: 175; color: "#0c1724"; border.color: "#2b4257"
                                Waveform { id: channelPlot; anchors.fill: parent; anchors.margins: 1; source: page.capture; channel: index }
                                Label { anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 5; text: modelData.name + "  [" + (modelData.unit || "原始单位") + "]  " + channelPlot.minimum.toPrecision(4) + " ～ " + channelPlot.maximum.toPrecision(4); color: modelData.color; font.pixelSize: 10 }
                                MouseArea { anchors.fill: parent; acceptedButtons: Qt.LeftButton | Qt.RightButton
                                    onClicked: function(mouse){page.capture.setCursor(mouse.button===Qt.LeftButton?0:1,mouse.x/width)}
                                    onWheel: function(wheel){page.capture.zoom(wheel.angleDelta.y>0?0.8:1.25,wheel.x/width);wheel.accepted=true}
                                }
                            }
                        }
                    }
                }
                Label { visible: !page.separateAxes; anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 7; text: plot.maximum.toPrecision(5); color: "#7793ac"; font.pixelSize: 9 }
                Label { visible: !page.separateAxes; anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 7; text: plot.minimum.toPrecision(5); color: "#7793ac"; font.pixelSize: 9 }
                Rectangle { visible: page.capture.cursorA >= 0 && page.capture.cursorA <= 1; x: page.capture.cursorA * plotFrame.width; width: 1; height: parent.height; color: "#dfedfa" }
                Rectangle { visible: page.capture.cursorB >= 0 && page.capture.cursorB <= 1; x: page.capture.cursorB * plotFrame.width; width: 1; height: parent.height; color: "#f3c675" }
                MouseArea {
                    objectName: "scopeInteraction"; enabled: !page.separateAxes; anchors.fill: parent; acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton; preventStealing: true
                    property real previousX: 0
                    onPressed: function(mouse) { previousX = mouse.x }
                    onPositionChanged: function(mouse) { if(mouse.buttons & Qt.MiddleButton) { page.capture.pan((previousX - mouse.x) / width); previousX = mouse.x } }
                    onClicked: function(mouse) { if(mouse.button !== Qt.MiddleButton) page.capture.setCursor(mouse.button === Qt.LeftButton ? 0 : 1, mouse.x / width) }
                    onWheel: function(wheel) { page.capture.zoom(wheel.angleDelta.y > 0 ? 0.8 : 1.25, wheel.x / width); wheel.accepted = true }
                }
            }
            RowLayout {
                Label { text: page.capture.viewStart.toFixed(6) + " s"; color: "#829bb7"; font.pixelSize: 10 }
                Item { Layout.fillWidth: true }
                Label { text: page.capture.viewEnd.toFixed(6) + " s · 设备时间"; color: "#829bb7"; font.pixelSize: 10 }
            }
            RowLayout {
                Label { text: "Y 范围"; color: "#829bb7"; font.pixelSize: 10 }
                TextField { id: low; text: "-5"; Layout.preferredWidth: 60; selectByMouse: true }
                TextField { id: high; text: "5"; Layout.preferredWidth: 60; selectByMouse: true }
                Button { text: "设定 Y"; enabled: !page.separateAxes; onClicked: plot.setVerticalRange(Number(low.text), Number(high.text)) }
                Item { Layout.fillWidth: true }
                Label { text: page.separateAxes ? "各通道独立自动 Y · 共用设备时间轴" : plot.renderer + " · " + plot.displayPoints + " 显示点 · 准备 " + plot.prepareMs.toFixed(2) + " ms"; color: "#7995af"; font.pixelSize: 9 }
            }
            ScrollView {
                Layout.fillWidth: true; Layout.preferredHeight: 58; clip: true
                Label { width: parent.width; text: page.capture.cursorText; wrapMode: Text.Wrap; color: "#b4c8db"; font.pixelSize: 10 }
            }
        }
    }
    Label {
        Layout.fillWidth: true; font.pixelSize: 10; color: "#829bb7"; elide: Text.ElideRight
        text: (page.capture.info.mode === 1 ? "连续" : "间断") + " · 已收 " + (page.capture.info.received || 0) + " 点 · " +
              (page.capture.info.segments || 0) + " 段 · 溢出 " + (page.capture.info.overflows || 0) + " · 历史 " +
              ((page.capture.info.memory || 0) / 1048576).toFixed(1) + " / 32 MiB · 已淘汰 " + (page.capture.info.evicted || 0) + " 块" +
              ((page.capture.info.throughput || 0) > 0 ? " · 测得负载 " + (page.capture.info.throughput / 1000).toFixed(0) + " kB/s" : "")
    }
}
