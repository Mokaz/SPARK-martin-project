#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint

import tkinter as tk
from tkinter import ttk

class SetpointGUI(Node):
    def __init__(self):
        super().__init__('setpoint_gui')

        # Create the publisher
        self.pub = self.create_publisher(TrajectorySetpoint, '/fmu/my_new_setpoints', 10)

        # 1) Initialize Tkinter Window
        self.root = tk.Tk()
        self.root.title("PX4 Setpoint Control")

        # 2) Some basic ttk styling to make it look nicer
        style = ttk.Style()
        # Use a theme like "clam", "default", "alt", or "classic"
        style.theme_use("clam")  
        style.configure("TLabel", font=("Helvetica", 12))
        style.configure("TEntry", font=("Helvetica", 12))
        style.configure("TButton", font=("Helvetica", 12))
        style.configure("Header.TLabel", font=("Helvetica", 14, "bold"))

        # 3) Title (Header)
        title_label = ttk.Label(self.root, text="PX4 Setpoint Control", style="Header.TLabel")
        title_label.pack(side=tk.TOP, pady=5)

        # 4) A labeled frame for our setpoint inputs
        input_frame = ttk.LabelFrame(self.root, text="Setpoint Inputs", padding="10 10 10 10")
        input_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Default values
        default_x = "0.0"
        default_y = "0.0"
        default_z = "-5.0"
        default_yaw = "0.0"

        # Row 1: X
        ttk.Label(input_frame, text="X Position:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.entry_x = ttk.Entry(input_frame, width=10)
        self.entry_x.insert(0, default_x)
        self.entry_x.grid(row=0, column=1, padx=5, pady=5)

        # Row 2: Y
        ttk.Label(input_frame, text="Y Position:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.entry_y = ttk.Entry(input_frame, width=10)
        self.entry_y.insert(0, default_y)
        self.entry_y.grid(row=1, column=1, padx=5, pady=5)

        # Row 3: Z
        ttk.Label(input_frame, text="Z Position:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.entry_z = ttk.Entry(input_frame, width=10)
        self.entry_z.insert(0, default_z)
        self.entry_z.grid(row=2, column=1, padx=5, pady=5)

        # Row 4: Yaw
        ttk.Label(input_frame, text="Yaw [rad]:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=5)
        self.entry_yaw = ttk.Entry(input_frame, width=10)
        self.entry_yaw.insert(0, default_yaw)
        self.entry_yaw.grid(row=3, column=1, padx=5, pady=5)

        # 5) Button Frame
        button_frame = ttk.Frame(self.root)
        button_frame.pack(side=tk.TOP, pady=5)

        # Publish Button
        publish_button = ttk.Button(
            button_frame, text="Publish Setpoint", command=self.publish_setpoint
        )
        publish_button.pack(side=tk.LEFT, padx=5)

        # Reset Button (optional: reset fields to defaults)
        reset_button = ttk.Button(
            button_frame, text="Reset", command=lambda: self.reset_fields(default_x, default_y, default_z, default_yaw)
        )
        reset_button.pack(side=tk.LEFT, padx=5)

        # Quit Button (close GUI & shutdown ROS)
        quit_button = ttk.Button(button_frame, text="Quit", command=self.quit_app)
        quit_button.pack(side=tk.LEFT, padx=5)

        # 6) Status Label at the bottom
        self.status_label = ttk.Label(self.root, text="No setpoint published yet", foreground="blue")
        self.status_label.pack(side=tk.BOTTOM, pady=5)

    def publish_setpoint(self):
        """Retrieve the input values and publish them as a TrajectorySetpoint."""
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())
            yaw = float(self.entry_yaw.get())
        except ValueError:
            self.status_label.config(text="Invalid numeric input!", foreground="red")
            return

        # Build the ROS2 message
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        # Publish
        self.pub.publish(msg)
        self.get_logger().info(f"Published setpoint: [x={x}, y={y}, z={z}] yaw={yaw}")

        # Update status text in the GUI
        self.status_label.config(
            text=f"Published setpoint: x={x:.1f}, y={y:.1f}, z={z:.1f}, yaw={yaw:.2f}",
            foreground="green"
        )

    def reset_fields(self, default_x, default_y, default_z, default_yaw):
        """Reset the entry fields to default values."""
        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, default_x)

        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, default_y)

        self.entry_z.delete(0, tk.END)
        self.entry_z.insert(0, default_z)

        self.entry_yaw.delete(0, tk.END)
        self.entry_yaw.insert(0, default_yaw)

        self.status_label.config(text="Fields reset to default", foreground="blue")

    def quit_app(self):
        """Quit the Tkinter app and cleanly shut down ROS."""
        self.root.quit()        # stop the Tk mainloop
        rclpy.shutdown()        # shut down ROS
        self.destroy_node()     # destroy the node

    def spin_tk(self):
        """Keep the Tkinter UI alive while handling ROS events."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.root.update_idletasks()
            self.root.update()

def main(args=None):
    rclpy.init(args=args)
    gui_node = SetpointGUI()
    gui_node.spin_tk()
    # If the user closes the window or hits Quit, the loop ends.
    # Make sure everything is shut down properly.
    if rclpy.ok():
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
