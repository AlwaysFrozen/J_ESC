import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

ColumnLayout {
    id: page
    property int groupId: 0
    spacing: 12
    FileDialog { id: saveDialog; title: "保存完整参数方案"; fileMode: FileDialog.SaveFile; nameFilters: ["J_ESC 参数方案 (*.jescconfig)"]; defaultSuffix: "jescconfig"; onAccepted: studio.saveScheme(selectedFile) }
    FileDialog { id: openDialog; title: "预览参数方案差异"; nameFilters: ["J_ESC 参数方案 (*.jescconfig)"]; onAccepted: { if(studio.previewScheme(selectedFile)) previewDialog.open() } }
    Dialog {
        id: previewDialog; title: "方案差异 · 载入后仅修改本地草稿"; modal: true; width: 760; height: 480
        anchors.centerIn: parent; standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: studio.applyScheme()
        contentItem: ListView { clip: true; model: studio.scheme.differences
            delegate: Label { required property var modelData; width: ListView.view.width; height: 42; color: "#d8e5f3"; elide: Text.ElideRight; text: modelData.name + "\n" + modelData.before + " → " + modelData.after + " " + modelData.unit }
        }
    }
    RowLayout {
        Layout.fillWidth: true
        Button { objectName: "readAll"; text: "读取全部"; enabled: studio.connected && !studio.busy; onClicked: studio.readAll() }
        Button { text: "保存方案"; enabled: !studio.busy; onClicked: saveDialog.open() }
        Button { text: "导入 / 比较方案"; enabled: !studio.busy; onClicked: openDialog.open() }
        Button { text: "写入全部修改"; enabled: studio.connected && !studio.busy && studio.dictionary.dirtyCount > 0; onClicked: studio.writeAllChanges() }
        Label { text: studio.scheme.message; Layout.fillWidth: true; elide: Text.ElideRight; color: "#91b2c6"; font.pixelSize: 10 }
    }
    RowLayout {
        Layout.fillWidth: true
        TextField { Layout.preferredWidth: 260; placeholderText: "搜索名称或说明"; placeholderTextColor: "#7891ad"; onTextChanged: studio.parameters.search = text }
        ComboBox { id: sectionPicker; model: ["全部分组"].concat(studio.parameters.sections); Layout.preferredWidth: 150; onActivated: studio.parameters.section = currentIndex === 0 ? "" : currentText; Connections { target: studio.parameters; function onGroupChanged(){sectionPicker.currentIndex = 0} } }
        Item { Layout.fillWidth: true }
        Button { objectName: "readGroup"; text: "读取本组"; enabled: studio.connected && !studio.busy; onClicked: studio.readGroup(page.groupId) }
        Button {
            text: "写入修改项"; enabled: studio.connected && !studio.busy && studio.dictionary.dirtyCount > 0
            objectName: "writeGroup"
            onClicked: { page.forceActiveFocus(); Qt.callLater(function(){studio.writeGroup(page.groupId)}) }
        }
        Button { objectName: "applyConfig"; text: "应用参数"; enabled: studio.connected && !studio.busy && studio.commandAvailable("apply"); onClicked: studio.command("apply") }
        Button { objectName: "saveFlash"; text: "保存 Flash"; enabled: studio.connected && !studio.busy && studio.commandAvailable("save"); onClicked: studio.command("save") }
    }
    Rectangle {
        Layout.fillWidth: true; Layout.preferredHeight: 38; radius: 5; color: "#1b2a3d"
        RowLayout {
            anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 16
            Label { text: "参数 / 结构体字段"; Layout.fillWidth: true; color: "#8fa8c4"; font.pixelSize: 11 }
            Label { text: "设备 RAM 值"; Layout.preferredWidth: 120; color: "#8fa8c4"; font.pixelSize: 11 }
            Label { text: "编辑值"; Layout.preferredWidth: 166; color: "#8fa8c4"; font.pixelSize: 11 }
            Label { text: "读回 / 撤销"; Layout.preferredWidth: 113; color: "#8fa8c4"; font.pixelSize: 11 }
        }
    }
    ListView {
        id: list
        objectName: "parameterList"
        Layout.fillWidth: true; Layout.fillHeight: true
        clip: true; spacing: 2; model: studio.parameters
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }
        delegate: Rectangle {
            required property int entryId
            required property string name
            required property string description
            required property string address
            required property string typeName
            required property string deviceValue
            required property string draftValue
            required property bool dirty
            required property string stateText
            required property string unitText
            required property string sectionName
            required property var enumOptions
            id: row
            width: list.width; height: 83; radius: 4
            color: dirty ? "#242a32" : (index % 2 ? "#141f2f" : "#162333")
            property int index: entryId
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 16
                ColumnLayout {
                    Layout.fillWidth: true; spacing: 3
                    Label { text: row.description || row.name; color: "#d8e5f3"; font.pixelSize: 12; elide: Text.ElideRight; Layout.fillWidth: true }
                    Label { text: row.name; color: "#859cb6"; font.pixelSize: 10; elide: Text.ElideRight; Layout.fillWidth: true }
                    Label { text: row.address + " · " + row.typeName + " · " + row.sectionName + " " + row.unitText + "   /   " + row.stateText; color: row.dirty ? "#ddba7b" : "#657f9d"; font.pixelSize: 9 }
                }
                Label { objectName: "device-" + row.name; text: row.deviceValue; Layout.preferredWidth: 120; color: "#84d7c0"; elide: Text.ElideRight; font.pixelSize: 13; font.family: "Consolas" }
                TextField {
                    id: editor
                    visible: row.enumOptions.length === 0
                    objectName: "editor-" + row.name
                    Layout.preferredWidth: 166; text: row.draftValue
                    enabled: !studio.busy; selectByMouse: true; placeholderText: "输入值"; placeholderTextColor: "#7891ad"
                    font.family: "Consolas"
                    onEditingFinished: { if(text !== row.draftValue) studio.dictionary.editValue(row.entryId,text); text = Qt.binding(function(){return row.draftValue}) }
                    onAccepted: page.forceActiveFocus()
                }
                ComboBox {
                    visible: row.enumOptions.length > 0; Layout.preferredWidth: 166; enabled: !studio.busy
                    model: row.enumOptions; textRole: "label"; valueRole: "value"
                    currentIndex: { for(let i=0;i<row.enumOptions.length;++i)if(row.enumOptions[i].value === row.draftValue)return i; return -1 }
                    displayText: currentIndex < 0 ? (row.draftValue.length ? "未知值 " + row.draftValue : "未读取") : currentText
                    onActivated: studio.dictionary.editValue(row.entryId, currentValue)
                }
                Button { text: "读回"; Layout.preferredWidth: 57; enabled: studio.connected && !studio.busy; onClicked: studio.readRow(row.entryId) }
                ToolButton { text: "撤销"; Layout.preferredWidth: 40; enabled: row.dirty && !studio.busy; onClicked: studio.dictionary.discard(row.entryId); ToolTip.visible: hovered; ToolTip.text: "撤销本地修改" }
            }
        }
        Label { anchors.centerIn: parent; visible: list.count === 0; text: "当前组没有匹配的参数"; color: "#829bb7" }
    }
    Label { text: "写入只修改 RAM。应用成功后可使能；持久化请单独保存 Flash。"; color: "#7f96b1"; font.pixelSize: 11; Layout.bottomMargin: 2 }
}

