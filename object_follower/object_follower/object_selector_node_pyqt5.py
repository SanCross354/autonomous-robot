import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QListWidget,
    QVBoxLayout, QMessageBox, QHBoxLayout
)
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ObjectSelectorNode(Node):
    def __init__(self):
        super().__init__('object_selector_gui')
        self.publisher_ = self.create_publisher(String, '/selected_object_class', 10)

    def publish_selection(self, selected_item):
        msg = String()
        msg.data = selected_item
        self.publisher_.publish(msg)
        if selected_item:
            self.get_logger().info(f"Published selected object: {selected_item}")
        else:
            self.get_logger().info("Published stop command (empty selection)")


class ObjectSelectorGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Object Selector GUI')
        self.setGeometry(300, 300, 300, 400)

        self.label = QLabel('Select an object to follow:')
        self.label.setAlignment(Qt.AlignCenter)

        self.object_list = QListWidget()
        self.object_list.addItems([
            'person', 'bottle', 'chair', 'cup', 'backpack', 'book', 'cell phone', 'laptop', 'remote'
        ])
        self.object_list.setSelectionMode(QListWidget.SingleSelection)

        self.select_button = QPushButton('Start Following')
        self.select_button.clicked.connect(self.on_select)

        self.stop_button = QPushButton('Stop Following')
        self.stop_button.clicked.connect(self.on_stop)

        # Layouts
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.select_button)
        button_layout.addWidget(self.stop_button)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.object_list)
        layout.addLayout(button_layout)

        self.setLayout(layout)

    def on_select(self):
        selected_items = self.object_list.selectedItems()
        if selected_items:
            selected_label = selected_items[0].text()
            self.ros_node.publish_selection(selected_label)
            QMessageBox.information(self, 'Selection Sent', f'Selected: {selected_label}')
        else:
            QMessageBox.warning(self, 'No Selection', 'Please select an object.')

    def on_stop(self):
        self.ros_node.publish_selection('')  # Send empty string
        QMessageBox.information(self, 'Stopped', 'Robot will stop following.')

def launch_gui():
    rclpy.init()
    ros_node = ObjectSelectorNode()

    app = QApplication(sys.argv)
    gui = ObjectSelectorGUI(ros_node)
    gui.show()

    timer = ros_node.create_timer(0.1, lambda: None)

    try:
        sys.exit(app.exec_())
    except SystemExit:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    launch_gui()
