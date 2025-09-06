#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, subprocess, signal, datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')

        # 조이스틱 버튼 파라미터 (PS5 예시: O=1, △=3)
        self.start_button = self.declare_parameter('start_button', 1).value
        self.stop_button  = self.declare_parameter('stop_button', 3).value

        # 레코딩 모드 파라미터
        # record_all=True면 `-a`(모든 토픽). False면 include_topics 목록만.
        self.record_all      = self.declare_parameter('record_all', True).value
        self.include_topics  = self.declare_parameter('include_topics', []).value  # 예: ['/scan','/tf','/tf_static']
        self.exclude_regex   = self.declare_parameter('exclude_regex', '').value   # 예: '.*image_raw.*|/parameter_events'
        self.storage_id      = self.declare_parameter('storage_id', 'sqlite3').value  # 'sqlite3' 또는 'mcap'
        self.compression_mode   = self.declare_parameter('compression_mode', '').value # ''|'file'|'message'
        self.compression_format = self.declare_parameter('compression_format', '').value # ''|'zstd'|'lz4'
        self.max_cache_size  = self.declare_parameter('max_cache_size', 0).value  # 0=기본, bytes

        # 저장 디렉토리
        self.save_dir = os.path.expanduser('~/forza_ws/race_stack/rosbag')
        os.makedirs(self.save_dir, exist_ok=True)

        # 내부 상태
        self.proc = None

        # /joy 구독
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.get_logger().info('[bag_recorder] ready. Press start/stop buttons to control recording.')

    def _build_cmd(self):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(self.save_dir, f"offline_{ts}")

        cmd = ['ros2', 'bag', 'record', '-o', out_path, '-s', self.storage_id]

        if self.record_all:
            cmd += ['-a']
            if self.exclude_regex:
                # ROS 2의 exclude 옵션: -x <regex>
                cmd += ['-x', self.exclude_regex]
        else:
            # 특정 토픽만
            if not self.include_topics:
                # 안전장치: 비워두면 기본 3종만
                self.include_topics = ['/scan', '/tf', '/tf_static']
            cmd += self.include_topics

        # 압축 옵션 (지원될 때만)
        if self.compression_mode:
            cmd += ['--compression-mode', self.compression_mode]
        if self.compression_format:
            cmd += ['--compression-format', self.compression_format]
        if isinstance(self.max_cache_size, int) and self.max_cache_size > 0:
            cmd += ['--max-cache-size', str(self.max_cache_size)]

        return cmd, out_path

    def _on_joy(self, msg: Joy):
        start_pressed = (self.start_button < len(msg.buttons) and msg.buttons[self.start_button] == 1)
        stop_pressed  = (self.stop_button  < len(msg.buttons) and msg.buttons[self.stop_button]  == 1)

        if start_pressed and self.proc is None:
            cmd, out_path = self._build_cmd()
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.get_logger().info(f"[bag_recorder] START recording → {out_path}")
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

