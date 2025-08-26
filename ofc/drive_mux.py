#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from ackermann_msgs.msg import AckermannDriveStamped

ALLOWED = ("tln", "ftg")

class DriveMux(Node):
    """
    /tln/drive 또는 /ftg/drive 중 하나만 /drive 로 재발행하는 멀티플렉서
    rqt Parameters에서 /drive_mux 노드의 'active' 파라미터 값을 'tln' 또는 'ftg'로 바꾸면 즉시 전환됨
    """
    def __init__(self):
        super().__init__("drive_mux")

        # 런타임 파라미터 선언 & 현재값 보관
        self.declare_parameter("active", "tln")  # "tln" 또는 "ftg"
        val = self.get_parameter("active").get_parameter_value().string_value
        self.active = val if val in ALLOWED else "tln"

        # 파라미터 변경 콜백
        self.add_on_set_parameters_callback(self._on_param_change)

        # 최종 출력 퍼블리셔 (/drive)
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # 각 소스 토픽 구독자
        self.sub_tln = self.create_subscription(
            AckermannDriveStamped, "/tln/drive", self._on_tln, 10
        )
        self.sub_ftg = self.create_subscription(
            AckermannDriveStamped, "/ftg/drive", self._on_ftg, 10
        )

        self.get_logger().info(f"[drive_mux] active='{self.active}' (tln/ftg)")

    def _on_param_change(self, params):
        for p in params:
            if p.name == "active" and p.type_ == Parameter.Type.STRING:
                if p.value in ALLOWED:
                    self.active = p.value
                    self.get_logger().info(f"[drive_mux] switched to '{self.active}'")
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().warn(f"[drive_mux] invalid value '{p.value}'")
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def _on_tln(self, msg: AckermannDriveStamped):
        if self.active == "tln":
            self.pub.publish(msg)

    def _on_ftg(self, msg: AckermannDriveStamped):
        if self.active == "ftg":
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = DriveMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

