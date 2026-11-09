import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    color: "#121f2e"; radius: 7
    ColumnLayout {
        anchors.fill: parent; anchors.margins: 12; spacing: 10
        RowLayout {
            Label { text: "轴 0 / 电机控制"; font.pixelSize: 16; font.bold: true; color: "#d9f4ee" }
            Item { Layout.fillWidth: true }
            Label { text: studio.connected ? "在线" : "离线"; color: studio.connected ? "#75e6ca" : "#8196b1" }
        }
        Button {
            objectName: "motorDisable"; text: "失能 / 停止"; Layout.fillWidth: true
            enabled: studio.connected && studio.commandAvailable("disable")
            palette.button: "#8c3e43"; onClicked: studio.command("disable")
        }
        RowLayout {
            Button { objectName: "motorEnable"; text: "使能"; Layout.fillWidth: true; enabled: studio.connected && !studio.controlBusy && studio.commandAvailable("enable"); onClicked: studio.command("enable") }
            Button { objectName: "motorFaultReset"; text: "故障复位"; Layout.fillWidth: true; enabled: studio.connected && !studio.controlBusy && studio.commandAvailable("fault_reset"); onClicked: studio.command("fault_reset") }
        }
        ScrollView {
            id: controlsScroll; Layout.fillWidth: true; Layout.fillHeight: true; clip: true
            contentWidth: availableWidth
            ColumnLayout {
                width: controlsScroll.availableWidth; spacing: 12
                Label { text: "运动目标"; color: "#c6deea"; font.bold: true }
                ComboBox { id: targetMode; objectName: "motionMode"; model: ["电流 / A", "速度 / 轴单位每秒", "位置 / 轴单位"]; Layout.fillWidth: true }
                GridLayout {
                    columns: 2; Layout.fillWidth: true; columnSpacing: 8; rowSpacing: 6
                    Label { text: "目标"; color: "#91aac3" }
                    TextField { id: target; objectName: "motionTarget"; text: "0"; selectByMouse: true; Layout.fillWidth: true; Layout.minimumWidth: 0 }
                    Label { text: "速度"; color: "#91aac3"; visible: targetMode.currentIndex===2 }
                    TextField { id: targetSpeed; text: "10"; visible: targetMode.currentIndex===2; selectByMouse: true; Layout.fillWidth: true; Layout.minimumWidth: 0 }
                    Label { text: "加速度"; color: "#91aac3"; visible: targetMode.currentIndex>0 }
                    TextField { id: targetAcc; text: "100"; visible: targetMode.currentIndex>0; selectByMouse: true; Layout.fillWidth: true; Layout.minimumWidth: 0 }
                    Label { text: "jerk"; color: "#91aac3"; visible: targetMode.currentIndex>0 }
                    TextField { id: targetJerk; text: "1000"; visible: targetMode.currentIndex>0; selectByMouse: true; Layout.fillWidth: true; Layout.minimumWidth: 0 }
                }
                Button { objectName: "motionSubmit"; text: "提交完整目标"; Layout.fillWidth: true; enabled: studio.connected && !studio.controlBusy && !!studio.motion.supported; onClicked: studio.submitMotion(targetMode.currentIndex+1,target.text,targetSpeed.text,targetAcc.text,targetJerk.text) }
                Button { objectName: "motionStop"; text: "停止在线控制"; Layout.fillWidth: true; enabled: studio.connected && !!studio.motion.supported; onClicked: studio.stopMotion() }
                Label { text: (studio.motion.state || "固件未提供运动接口") + "  " + (studio.motion.error || ""); Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#85d8c0" }
                Label { text: "旋转轴用 deg，直线轴用 mm；速度、加速度和 jerk 为相应时间导数。须使能、校准且寻相完成，提交目标不会自动使能。在线心跳超时 500 ms 将失能。"; Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#91aac3"; font.pixelSize: 10 }
                Rectangle { Layout.fillWidth: true; height: 1; color: "#29374a" }
                Label { text: "寻相与辨识"; color: "#c6deea"; font.bold: true }
                Button { objectName: "motorAlign"; text: "启动寻相"; Layout.fillWidth: true; enabled: studio.connected && !studio.controlBusy && studio.commandAvailable("align"); onClicked: studio.command("align") }
                RowLayout {
                    ComboBox { id: identify; model: ["电阻 Rs", "电感 Ld/Lq", "极对数", "磁链"]; Layout.fillWidth: true; Layout.minimumWidth: 0; enabled: !studio.controlBusy }
                    Button { text: "启动辨识"; enabled: studio.connected && !studio.controlBusy && studio.commandAvailable("identify"); onClicked: studio.command("identify", identify.currentIndex+1) }
                }
                Label { text: "使能后进入无运动模式。手动寻相、辨识不受在线心跳保护，结束后请显式失能。"; Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#d6bc8b"; font.pixelSize: 10 }
                RowLayout {
                    Label { text: "设备状态"; color: "#c6deea"; font.bold: true }
                    Item { Layout.fillWidth: true }
                    Button { text: "刷新"; enabled: studio.connected && !studio.controlBusy; onClicked: studio.refreshStatus() }
                }
                GridLayout {
                    columns: 2; columnSpacing: 6; rowSpacing: 6; Layout.fillWidth: true
                    Repeater {
                        model: studio.motorStatus
                        delegate: Rectangle {
                            required property var modelData
                            Layout.fillWidth: true; Layout.preferredHeight: 58; Layout.minimumWidth: 0
                            radius: 5; color: "#172435"; border.color: "#26384d"
                            ColumnLayout {
                                anchors.fill: parent; anchors.margins: 8; spacing: 2
                                Label { text: modelData.label; color: "#8ba2bd"; font.pixelSize: 10 }
                                Label { text: modelData.value; Layout.fillWidth: true; elide: Text.ElideRight; color: "#d6ede7"; font.pixelSize: 16; font.family: "Consolas" }
                            }
                        }
                    }
                }
                Label { text: "状态每秒刷新。命令确认后，请结合阶段与故障状态判断是否完成。"; Layout.fillWidth: true; wrapMode: Text.Wrap; color: "#8ba2bd"; font.pixelSize: 10 }
            }
        }
    }
}
