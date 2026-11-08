import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

ApplicationWindow {
    id: window
    visible: true
    width: 1460; height: 940
    minimumWidth: 1120; minimumHeight: 740
    title: "J_ESC Studio · 电机调试"
    color: "#0d141f"
    palette.window: "#111c2b"
    palette.base: "#0d1623"
    palette.button: "#23344b"
    palette.buttonText: "#e4edf8"
    palette.text: "#e4edf8"
    palette.windowText: "#e4edf8"
    palette.highlight: "#35c9aa"
    palette.highlightedText: "#0d141f"
    property int selectedPage: 0
    readonly property var pageTitles: ["电机参数", "编码器参数", "电机控制器", "观测器", "控制环", "环路滤波器", "HFI", "电机控制", "高速示波器"]
    readonly property var structs: ["Motor_Config_t", "Encoder_Config_t", "Motor_Controller_Config_t", "Motor_Observer_Config_t", "Loop_Controller_Config_t", "Loop_Filter_Config_t", "HFI_Config_t"]
    onSelectedPageChanged: studio.parameters.group = Math.min(selectedPage, 6)
    FileDialog {
        id: dictionaryDialog
        title: "载入固件对应的对象字典"
        nameFilters: ["对象字典 (*.csv)"]
        onAccepted: studio.loadDictionary(selectedFile)
    }
    RowLayout {
        anchors.fill: parent; spacing: 0
        Rectangle {
            Layout.preferredWidth: 226; Layout.fillHeight: true
            color: "#121d2c"
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 18; spacing: 8
                Label { text: "J_ESC"; font.pixelSize: 30; font.bold: true; color: "#edf7ff"; Layout.topMargin: 10 }
                Label { text: "MOTOR DEVELOPMENT STUDIO"; Layout.fillWidth: true; elide: Text.ElideRight; font.pixelSize: 9; font.letterSpacing: 1; color: "#7f94b0" }
                Rectangle { Layout.fillWidth: true; height: 1; color: "#29374a"; Layout.topMargin: 20; Layout.bottomMargin: 12 }
                Label { text: "轴 0 / 配置"; color: "#7f94b0"; font.pixelSize: 11; Layout.bottomMargin: 5 }
                Repeater {
                    model: window.pageTitles
                    delegate: ItemDelegate {
                        required property string modelData
                        required property int index
                        Layout.fillWidth: true; Layout.preferredHeight: 43
                        text: modelData
                        objectName: "page-" + index
                        highlighted: window.selectedPage === index
                        onClicked: window.selectedPage = index
                        background: Rectangle { radius: 6; color: parent.highlighted ? "#203f45" : (parent.hovered ? "#1c2c40" : "transparent") }
                        contentItem: Label { text: parent.text; color: parent.highlighted ? "#75e6ca" : "#b9c9dd"; verticalAlignment: Text.AlignVCenter; leftPadding: 12 }
                    }
                }
                Item { Layout.fillHeight: true }
                Label { text: "本地修改"; color: "#8196b1"; font.pixelSize: 11 }
                Label { text: studio.dictionary.dirtyCount + " 个参数"; font.pixelSize: 23; color: studio.dictionary.dirtyCount ? "#efc078" : "#b5c9dd" }
                Label { text: "RAM 写入 → 应用 → Flash 保存"; Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#8196b1"; font.pixelSize: 10; Layout.bottomMargin: 12 }
            }
        }
        ColumnLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 0
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: 82; color: "#162131"
                RowLayout {
                    anchors.fill: parent; anchors.margins: 20; spacing: 10
                    Rectangle { width: 8; height: 8; radius: 4; color: studio.connected ? "#41d6b1" : "#6e829d" }
                    ColumnLayout {
                        spacing: 2
                        Label { text: studio.connectionLabel; font.bold: true; color: "#e0edf8" }
                        Label { text: studio.connecting ? "正在协商 USB 协议…" : (studio.connected ? "USB CDC · 协议 v2" : "连接设备开始调试"); font.pixelSize: 10; color: "#879db8" }
                    }
                    Item { Layout.fillWidth: true }
                    ComboBox { id: ports; model: studio.ports; editable: true; Layout.preferredWidth: 120; enabled: !studio.connected && !studio.connecting; displayText: currentIndex < 0 ? "COM 端口" : currentText }
                    Button { text: "刷新"; enabled: !studio.connecting; onClicked: studio.refreshPorts() }
                    Button { text: studio.connected || studio.connecting ? "断开" : "连接"; onClicked: studio.connected || studio.connecting ? studio.disconnectPort() : studio.connectPort(ports.editText) }
                    Button { objectName: "openDemo"; text: "演示设备"; enabled: !studio.connected && !studio.connecting; onClicked: studio.openDemo() }
                    Button { text: "载入字典"; enabled: !studio.connected && !studio.connecting && studio.dictionary.dirtyCount === 0; onClicked: dictionaryDialog.open() }
                }
            }
            RowLayout {
                Layout.fillWidth: true; Layout.leftMargin: 28; Layout.rightMargin: 28; Layout.topMargin: 24; Layout.bottomMargin: 18
                ColumnLayout {
                    Label { text: window.pageTitles[window.selectedPage]; font.pixelSize: 27; font.bold: true; color: "#edf5ff" }
                    Label { text: window.selectedPage < 7 ? window.structs[window.selectedPage] : (window.selectedPage === 7 ? "使能、寻相、辨识与设备状态" : "控制周期采样 · 设备时间轴 · 场景图波形"); color: "#8b9fb9"; font.pixelSize: 11 }
                }
                Item { Layout.fillWidth: true }
                Label { text: studio.configState; color: "#87d8c2"; font.pixelSize: 11 }
            }
            StackLayout {
                Layout.fillWidth: true; Layout.fillHeight: true
                Layout.leftMargin: 28; Layout.rightMargin: 28
                currentIndex: window.selectedPage < 7 ? 0 : window.selectedPage - 6
                ParameterPage { groupId: Math.min(window.selectedPage,6) }
                MotorPage {}
                ScopePage {}
            }
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: 101
                Layout.leftMargin: 28; Layout.rightMargin: 28; Layout.topMargin: 16; Layout.bottomMargin: 14
                color: "#131e2d"; radius: 7; border.color: "#253448"
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 12; spacing: 4
                    RowLayout {
                        Label { text: "设备消息"; font.pixelSize: 11; color: "#839ab5" }
                        Item { Layout.fillWidth: true }
                        Label { text: studio.pendingCount ? (studio.busy ? studio.pendingCount + " 个请求处理中" : "刷新状态") : "空闲"; color: studio.busy ? "#efc078" : "#75cdb3"; font.pixelSize: 10 }
                    }
                    Label { text: studio.message; wrapMode: Text.Wrap; Layout.fillWidth: true; font.pixelSize: 12; color: "#c7d7e9" }
                    Label { text: "字典：" + studio.dictionary.source; elide: Text.ElideMiddle; Layout.fillWidth: true; color: "#6f87a4"; font.pixelSize: 10 }
                }
            }
        }
    }
}
