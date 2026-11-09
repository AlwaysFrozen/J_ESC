import QtQuick
import QtQuick.Controls

Item {
    id: interaction
    required property var capture
    required property var waveform
    property bool crosshairEnabled: false
    property string unit: ""
    readonly property bool crosshairVisible: crosshairEnabled && pointer.containsMouse
    readonly property real hoverTime: capture.viewStart + pointer.mouseX / Math.max(1, width) * capture.span
    readonly property real hoverValue: waveform.maximum - pointer.mouseY / Math.max(1, height) * (waveform.maximum - waveform.minimum)

    Item {
        objectName: interaction.objectName + "Crosshair"
        anchors.fill: parent
        visible: interaction.crosshairVisible
        Rectangle { x: pointer.mouseX; width: 1; height: parent.height; color: "#c1dbed"; opacity: 0.8 }
        Rectangle { y: pointer.mouseY; width: parent.width; height: 1; color: "#c1dbed"; opacity: 0.8 }
        Rectangle {
            width: Math.min(readout.implicitWidth + 16, interaction.width - 8); height: readout.implicitHeight + 10
            x: Math.max(4, Math.min(pointer.mouseX + 12, interaction.width - width - 4))
            y: Math.max(4, pointer.mouseY + height + 16 < interaction.height ? pointer.mouseY + 12 : pointer.mouseY - height - 12)
            color: "#182b3e"; border.color: "#53758f"; radius: 4
            Label {
                id: readout; anchors.fill: parent; anchors.margins: 5; elide: Text.ElideRight
                text: "t " + interaction.hoverTime.toFixed(6) + " s  ·  Y " + interaction.hoverValue.toPrecision(6) + (interaction.unit ? " " + interaction.unit : "")
                color: "#e0edf8"; font.pixelSize: 10
            }
        }
    }
    MouseArea {
        id: pointer
        anchors.fill: parent; hoverEnabled: true
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        preventStealing: true
        cursorShape: dragging ? Qt.ClosedHandCursor : (interaction.crosshairEnabled ? Qt.CrossCursor : Qt.ArrowCursor)
        property bool dragging: false
        property bool verticalDrag: false
        property real pressX: 0
        property real pressY: 0
        property real previousX: 0
        property real startLow: 0
        property real startHigh: 1
        onPressed: function(mouse) {
            dragging = false; verticalDrag = false
            pressX = previousX = mouse.x; pressY = mouse.y
            startLow = interaction.waveform.minimum; startHigh = interaction.waveform.maximum
        }
        onPositionChanged: function(mouse) {
            if (!(mouse.buttons & (Qt.LeftButton | Qt.MiddleButton))) return
            let dx = mouse.x - pressX, dy = mouse.y - pressY
            if (!dragging && Math.hypot(dx, dy) < 6) return
            dragging = true
            interaction.capture.pan((previousX - mouse.x) / Math.max(1, width))
            previousX = mouse.x
            if (Math.abs(dy) >= 6) verticalDrag = true
            if (verticalDrag) {
                let offset = dy / Math.max(1, height) * (startHigh - startLow)
                interaction.waveform.setVerticalRange(startLow + offset, startHigh + offset)
            }
        }
        onReleased: function(mouse) {
            if (!dragging && mouse.button !== Qt.MiddleButton)
                interaction.capture.setCursor(mouse.button === Qt.LeftButton ? 0 : 1, mouse.x / Math.max(1, width))
            dragging = false; verticalDrag = false
        }
        onCanceled: { dragging = false; verticalDrag = false }
        onWheel: function(wheel) {
            interaction.capture.zoom(wheel.angleDelta.y > 0 ? 0.8 : 1.25, wheel.x / Math.max(1, width))
            wheel.accepted = true
        }
    }
}
