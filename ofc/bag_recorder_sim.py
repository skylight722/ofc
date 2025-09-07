#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy, os, subprocess, signal, datetime
from rclpy.node import Node
from sensor_msgs.msg import Joy

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder_sim')

        # 버튼 인덱�
        self.start_button = self.declare_parameter('start_button', 1).value
        self.stop_button  = self.declare_parameter('stop_button', 3).value

        # 저장 경로
        self.proc = None
        self.save_dir = os.path.expanduser('~/forza_ws/race_stack/rosbag')
        os.makedirs(self.save_dir, exist_ok=True)

        # 입력 구독
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.get_logger().info('[bag_recorder] ready. Press start/stop buttons to control recording.')

    def _on_joy(self, msg: Joy):
        start_pressed = (self.start_button < len(msg.buttons) and msg.buttons[self.start_button] == 1)
        stop_pressed  = (self.stop_button  < len(msg.buttons) and msg.buttons[self.stop_button]  == 1)

        if start_pressed and self.proc is None:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            out_path = os.path.join(self.save_dir, f"offline_{ts}")
            cmd = ["ros2", "bag", "record", "-o", out_path, "/scan", "/tf", "/tf_static"]
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.get_logger().info(f"[bag_recorder] START recording → {out_path}")

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

