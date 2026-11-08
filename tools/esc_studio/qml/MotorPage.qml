import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ColumnLayout {
    spacing: 16
    Rectangle {
        Layout.fillWidth: true; Layout.preferredHeight: 104; radius: 8; color: "#152838"; border.color: "#244c55"
        RowLayout {
            anchors.fill: parent; anchors.margins: 20; spacing: 12
            ColumnLayout {
                Layout.fillWidth: true
                Label { text: "轴 0"; font.pixelSize: 20; font.bold: true; color: "#d9f4ee" }
                Label { text: "使能后进入无运动模式，寻相与辨识需要单独启动。"; color: "#86acae"; font.pixelSize: 11 }
            }
            Button { objectName: "motorEnable"; text: "使能"; enabled: studio.connected && !studio.busy && studio.commandAvailable("enable"); onClicked: studio.command("enable") }
            Button {
                text: "失能 / 停止"; enabled: studio.connected && studio.commandAvailable("disable")
                objectName: "motorDisable"
                palette.button: "#8c3e43"; onClicked: studio.command("disable")
            }
            Button { text: "故障复位"; enabled: studio.connected && !studio.busy && studio.commandAvailable("fault_reset"); onClicked: studio.command("fault_reset") }
        }
    }
    RowLayout {
        spacing: 12
        Button { text: "启动寻相"; enabled: studio.connected && !studio.busy && studio.commandAvailable("align"); onClicked: studio.command("align") }
        ComboBox { id: identify; model: ["电阻 Rs", "电感 Ld/Lq", "极对数", "磁链"]; Layout.preferredWidth: 155; enabled: !studio.busy }
        Button { text: "启动辨识"; enabled: studio.connected && !studio.busy && studio.commandAvailable("identify"); onClicked: studio.command("identify", identify.currentIndex+1) }
        Item { Layout.fillWidth: true }
        Button { text: "刷新状态"; enabled: studio.connected && !studio.busy; onClicked: studio.refreshStatus() }
    }
    Rectangle {
        Layout.fillWidth: true; Layout.preferredHeight: 160; color: "#172435"; radius: 6
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 12
            Label { text: "基础运动目标 · 接口已接入，真实电机尚未验收"; color: "#c6deea" }
            RowLayout {
                ComboBox { id: targetMode; objectName: "motionMode"; model: ["电流 A", "速度", "位置"]; Layout.preferredWidth: 130 }
                TextField { id: target; objectName: "motionTarget"; text: "0"; placeholderText: "目标"; Layout.preferredWidth: 115 }
                TextField { id: targetSpeed; text: "10"; enabled: targetMode.currentIndex===2; placeholderText: "速度"; Layout.preferredWidth: 115 }
                TextField { id: targetAcc; text: "100"; enabled: targetMode.currentIndex>0; placeholderText: "加速度"; Layout.preferredWidth: 115 }
                TextField { id: targetJerk; text: "1000"; enabled: targetMode.currentIndex>0; placeholderText: "jerk"; Layout.preferredWidth: 115 }
                Button { objectName: "motionSubmit"; text: "提交完整目标"; enabled: studio.connected && !studio.busy && !!studio.motion.supported; onClicked: studio.submitMotion(targetMode.currentIndex+1,target.text,targetSpeed.text,targetAcc.text,targetJerk.text) }
                Button { objectName: "motionStop"; text: "停止在线控制"; enabled: studio.connected && !!studio.motion.supported; onClicked: studio.stopMotion() }
            }
            Label { text: "旋转轴：deg、deg/s；直线轴：mm、mm/s。加速度、jerk 分别为相应轴单位/s²、/s³。"; color: "#91aac3"; font.pixelSize: 10 }
            Label { text: "须已使能、校准且寻相完成。只提交目标，不自动使能。在线控制心跳超时 500 ms 或断线将失能。"; color: "#d6bc8b"; font.pixelSize: 10 }
            Label { text: (studio.motion.state || "固件未提供运动接口") + "  " + (studio.motion.error || ""); color: "#85d8c0" }
        }
    }
    GridLayout {
        columns: 4; columnSpacing: 12; rowSpacing: 12; Layout.fillWidth: true
        Repeater {
            model: studio.motorStatus
            delegate: Rectangle {
                required property var modelData
                Layout.fillWidth: true; Layout.preferredHeight: 72
                radius: 7; color: "#172435"; border.color: "#26384d"
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 12; spacing: 4
                    Label { text: modelData.label; color: "#8ba2bd"; font.pixelSize: 11 }
                    Label { text: modelData.value; color: "#d6ede7"; font.pixelSize: 18; font.family: "Consolas" }
                }
            }
        }
    }
    Label { text: "状态每秒后台刷新。命令确认表示已接收或已启动，完成结果请结合阶段与故障状态判断。"; wrapMode: Text.Wrap; Layout.fillWidth: true; color: "#8ba2bd"; font.pixelSize: 11 }
    Item { Layout.fillHeight: true }
}
