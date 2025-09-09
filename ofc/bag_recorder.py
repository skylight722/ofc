#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, subprocess, signal, datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')

        # Button parameters
        self.start_button = self.declare_parameter('start_button', 2).value
        self.stop_button  = self.declare_parameter('stop_button', 0).value

        # Recording parameters
        self.record_all     = self.declare_parameter('record_all', True).value
        self.include_topics = self.declare_parameter('include_topics', []).value
        self.exclude_regex  = self.declare_parameter('exclude_regex', '').value
        self.storage_id     = self.declare_parameter('storage_id', 'sqlite3').value

        # Save directory
        self.save_dir = os.path.expanduser('~/forza_ws/race_stack/rosbag')
        os.makedirs(self.save_dir, exist_ok=True)

        # Current recording process
        self.proc = None

        # Subscribe joystick
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.get_logger().info('[bag_recorder] ready. Press start/stop buttons to control recording.')

    def _build_cmd(self):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(self.save_dir, f"offline_{ts}")

        cmd = ['ros2', 'bag', 'record', '-o', out_path, '-s', self.storage_id]

        if self.record_all:
            cmd += ['-a']
            if self.exclude_regex:
                cmd += ['-x', self.exclude_regex]  # exclude by regex
        else:
            # Default minimal topics
            topics = self.include_topics or ['/scan', '/tf', '/tf_static']
            cmd += topics

        return cmd, out_path

    def _on_joy(self, msg: Joy):
        start_pressed = (self.start_button < len(msg.buttons) and msg.buttons[self.start_button] == 1)
        stop_pressed  = (self.stop_button  < len(msg.buttons) and msg.buttons[self.stop_button]  == 1)

        if start_pressed and self.proc is None:
            cmd, out_path = self._build_cmd()
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.get_logger().info(f"[bag_recorder] START recording -> {out_path}")
            self.get_logger().info(f"[bag_recorder] CMD: {' '.join(cmd)}")

        elif stop_pressed and self.proc is not None:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc = None
            self.get_logger().info("[bag_recorder] STOP recording")

def main(args=None):
    rclpy.init(args=args)
    node = BagRecorder()
    try:
        rclpy.spin(node)
    finally:
        if node.proc is not None:
            os.killpg(os.getpgid(node.proc.pid), signal.SIGINT)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



