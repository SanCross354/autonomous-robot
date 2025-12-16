# object_follower/object_selector_gui.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk

class ObjectSelectorGUI(Node):
    def __init__(self):
        super().__init__('object_selector_gui')
        self.publisher = self.create_publisher(String, '/selected_object_class', 10)

        self.selected = set()
        self.objects = ['person', 'cup', 'bottle', 'chair', 'tv', 'cell phone', 'book']

        self.root = tk.Tk()
        self.root.title("Select Object(s) to Follow")

        self.vars = {}

        for obj in self.objects:
            var = tk.IntVar()
            chk = tk.Checkbutton(self.root, text=obj, variable=var)
            chk.pack(anchor='w')
            self.vars[obj] = var

        start_button = tk.Button(self.root, text="Start Following", command=self.publish_selection)
        start_button.pack(pady=10)

    def publish_selection(self):
        selected_objects = [obj for obj, var in self.vars.items() if var.get()]
        msg = String()
        msg.data = ",".join(selected_objects)
        self.publisher.publish(msg)
        print(f"[GUI] Published: {msg.data}")

    def launch_gui(self):
        self.root.mainloop()

def launch_gui():
    rclpy.init()
    node = ObjectSelectorGUI()
    node.launch_gui()
    rclpy.shutdown()
