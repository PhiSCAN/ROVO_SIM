import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

ColumnLayout {
  // Tall enough that the yaw row cannot be squeezed off the bottom of the
  // docked card, which is what hid it before.
  Layout.minimumWidth: 320
  Layout.minimumHeight: 300
  anchors.fill: parent
  anchors.margins: 10
  spacing: 6

  RowLayout {
    Label { text: "Model" }
    TextField {
      id: modelField
      Layout.fillWidth: true
      text: RovoTeleport.modelName
      onEditingFinished: RovoTeleport.modelName = text
    }
  }

  Button {
    id: pickButton
    Layout.fillWidth: true
    checkable: true
    checked: RovoTeleport.picking
    text: checked ? "Click a point in the scene..." : "Pick pose in scene"
    onClicked: RovoTeleport.picking = checked
    ToolTip.visible: hovered
    ToolTip.text: "Then click the ground. The model lands on whatever surface " +
                  "you clicked, keeping its current heading."
  }

  Button {
    id: followButton
    Layout.fillWidth: true
    checkable: true
    checked: RovoTeleport.following
    text: checked ? "Camera: locked on model" : "Camera: free"
    onClicked: RovoTeleport.following = checked
    ToolTip.visible: hovered
    ToolTip.text: "Lock or release the chase camera. Nothing re-applies it " +
                  "behind your back, so releasing it stays released."
  }

  MenuSeparator { Layout.fillWidth: true }

  RowLayout {
    Layout.fillWidth: true
    Label { text: "Yaw" }
    TextField {
      id: yawField
      Layout.fillWidth: true
      text: "0.0"
      placeholderText: "degrees"
      validator: DoubleValidator {}
    }
    Button {
      text: "Turn"
      onClicked: RovoTeleport.SetHeading(parseFloat(yawField.text || "0"))
      ToolTip.visible: hovered
      ToolTip.text: "Turn in place; position is left alone."
    }
  }

  Button {
    Layout.fillWidth: true
    text: "Reset to spawn"
    onClicked: RovoTeleport.ResetPose()
  }

  Label {
    Layout.fillWidth: true
    wrapMode: Text.Wrap
    text: RovoTeleport.statusText
  }

  Item { Layout.fillHeight: true }
}
