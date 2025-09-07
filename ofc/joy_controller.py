#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult

class OfcJoyController(Node):
    def __init__(self):
        super().__init__('ofc_joy_controller')

        # 파라미터 선언
        self.declare_parameters('', [
            ('drive_topic', '/drive'),
            ('deadman_button', 4),    # PS5 L1
            ('axis_steer', 3),        # PS5 오른쪽 스틱 X축
            ('scale_steer', 0.34),    # 조향 각도 최대값 (rad)
            ('deadzone', 0.05),       # 조향 데드존
            ('fixed_speed', 3.0),     # 고정 속도 (m/s)
            ('publish_hz', 50.0),     # 발행 주기 (Hz)
            ('drive_publish_enabled', False),
        ])

        p = self.get_parameter
        self.drive_topic     = p('drive_topic').value
        self.deadman_button  = int(p('deadman_button').value)
        self.axis_steer      = int(p('axis_steer').value)
        self.scale_steer     = float(p('scale_steer').value)
        self.deadzone        = float(p('deadzone').value)
        self.fixed_speed     = float(p('fixed_speed').value)
        self.drive_publish_enabled = bool(p('drive_publish_enabled').value)

        # 구독 및 퍼블리셔
        self.sub_joy   = self.create_subscription(Joy, '/joy', self._on_joy, 50)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        # 내부 상태
        self._last_joy = None
        self._last_deadman = 0
        self.deadman_active = False
        self._last_pub_t = 0.0
        self._pub_dt = 1.0 / float(self.get_parameter('publish_hz').value)

        self.add_on_set_parameters_callback(self._on_set_params)
        self.timer = self.create_timer(self._pub_dt, self._tick)
        self.get_logger().info('[ofc_joy_controller] ready (fixed speed + deadman mode)')

    def _on_set_params(self, params):
        for prm in params:
            if prm.name == 'drive_publish_enabled':
                self.drive_publish_enabled = bool(prm.value)
            elif prm.name == 'publish_hz':
                self._pub_dt = 1.0 / float(prm.value)
                self.timer.timer_period_ns = int(self._pub_dt * 1e9)
            elif prm.name == 'fixed_speed':
                self.fixed_speed = float(prm.value)
            elif prm.name == 'scale_steer':
                self.scale_steer = float(prm.value)
            elif prm.name == 'deadzone':
                self.deadzone = float(prm.value)
            elif prm.name == 'axis_steer':
                self.axis_steer = int(prm.value)
        return SetParametersResult(successful=True)

    def _on_joy(self, msg: Joy):
        self._last_joy = msg
        deadman = self._btn(msg, self.deadman_button)

        # 버튼을 놓을 때 0,0 명령을 발행하여 안전 정지
        if self._last_deadman == 1 and deadman == 0 and self.drive_publish_enabled:
            self._publish(0.0, 0.0)

        self.deadman_active = (deadman == 1)
        self._last_deadman = deadman

    def _tick(self):
        # 파라미터가 허용된 상태이고 버튼이 눌렸을 때만 발행
        if not (self.drive_publish_enabled and self.deadman_active):
            return
        if self._last_joy is None:
            return
        now = time.monotonic()
        if now - self._last_pub_t < self._pub_dt:
            return
        self._last_pub_t = now

        steer = self._map_steer(self._last_joy)
        self._publish(self.fixed_speed, steer)

    # 조향 입력 처리
    def _map_steer(self, j: Joy) -> float:
        raw = self._axis(j, self.axis_steer)
        val = 0.0 if abs(raw) < self.deadzone else raw
        val = max(-1.0, min(1.0, val)) * self.scale_steer
        return float(val)

    # 메시지 발행
    def _publish(self, speed: float, steer: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub_drive.publish(msg)

    @staticmethod
    def _axis(j: Joy, idx: int) -> float:
        return float(j.axes[idx]) if 0 <= idx < len(j.axes) else 0.0

    @staticmethod
    def _btn(j: Joy, idx: int) -> int:
        return int(j.buttons[idx]) if 0 <= idx < len(j.buttons) else 0

def main(args=None):
    rclpy.init(args=args)
    node = OfcJoyController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

