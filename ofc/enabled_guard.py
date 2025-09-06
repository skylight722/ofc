#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent, Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from sensor_msgs.msg import Joy
from typing import List, Optional

class EnabledGuard(Node):
    """
    기존 기능:
      - targets 중 한 노드의 'drive_publish_enabled'가 True로 바뀌면, 나머지를 False로 내림
      - 시작 시 둘 이상 True면 prefer만 남김

    최소 추가:
      - /joy 구독
      - deadman 버튼 + (우스틱 X축 활동) -> 조이스틱 노드만 ON, targets 전부 OFF
      - return 버튼(rising) -> 조이스틱 노드 OFF, 마지막 자동 노드(last_active) ON
    """
    def __init__(self):
        super().__init__('enabled_guard')

        # ---- 기존 파라미터 (그대로) ----
        self.targets: List[str] = list(self.declare_parameter(
            'targets', ['tln_inference', 'ftg_controller']
        ).get_parameter_value().string_array_value)

        self.prefer: str = self.declare_parameter(
            'prefer', 'ofc_joy_controller'
        ).get_parameter_value().string_value

        # ---- 조이스틱 노드 이름 ----
        self.joy_node: str = self.declare_parameter(
            'joy_node', 'ofc_joy_controller'
        ).get_parameter_value().string_value

        # ---- 조이스틱 입력 파라미터 ----
        self.deadman_button: int = self.declare_parameter(
            'deadman_button', 4  # PS5 L1
        ).get_parameter_value().integer_value

        self.return_button: int = self.declare_parameter(
            'return_button', 5   # PS5 R1
        ).get_parameter_value().integer_value

        self.axis_deadzone: float = self.declare_parameter(
            'axis_deadzone', 0.05
        ).get_parameter_value().double_value

        # ---- 우스틱 X축만 활동으로 인정 (기본 3: 흔히 우스틱 X축) ----
        self.right_x_axis: int = self.declare_parameter(
            'right_x_axis', 3
        ).get_parameter_value().integer_value

        # ---- 상태 ----
        self._did_check = False
        self.manual_active: bool = False
        self.last_active: Optional[str] = self.prefer
        self._prev_btn: dict[int,int] = {}

        # ---- 구독 ----
        self.create_subscription(ParameterEvent, '/parameter_events', self._on_event, 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 50)

        self.get_logger().info(
            f'[enabled_guard] targets={self.targets}, prefer={self.prefer}, joy_node={self.joy_node}, right_x_axis={self.right_x_axis}, deadzone={self.axis_deadzone}'
        )

        # ---- 시작 시 정합(그대로) ----
        self.create_timer(1.0, self._startup_check_once)

    # ========== 시작 정합 ==========
    def _startup_check_once(self):
        if self._did_check:
            return
        self._did_check = True

        # ---- 시작은 무조건 조이스틱 모드 ----
        for n in self.targets:
            self._set_enabled(n, False)
        self._set_enabled(self.joy_node, True)
        self.manual_active = True
        self.last_active = self.joy_node
        self.get_logger().info(f"[enabled_guard] startup -> joy_node {self.joy_node}=True, targets OFF")

    # ========== parameter_events 처리 ==========
    def _on_event(self, evt: ParameterEvent):
        node_base = evt.node.split('/')[-1] if '/' in evt.node else evt.node
        # targets 또는 joy_node 외에는 무시
        if node_base not in self.targets and node_base != self.joy_node:
            return

        for p in list(evt.new_parameters) + list(evt.changed_parameters):
            if p.name != 'drive_publish_enabled':
                continue
            if p.value.type == ParameterType.PARAMETER_BOOL and p.value.bool_value:
                # 어떤 노드가 True가 됨
                if node_base in self.targets:
                    # 자동 노드가 True → 나머지 자동 노드 OFF
                    for other in self.targets:
                        if other != node_base:
                            self._set_enabled(other, False)
                    # 복귀용으로 기억
                    self.last_active = node_base
                    self.get_logger().info(f'[enabled_guard] {node_base}=True -> others OFF (last_active={self.last_active})')
                elif node_base == self.joy_node:
                    # 조이스틱이 True → 자동 노드 전부 OFF (논리 일관)
                    for other in self.targets:
                        self._set_enabled(other, False)
                    self.get_logger().info('[enabled_guard] joy_node=True -> all targets OFF')
                break

    # ========== 조이스틱 이벤트 ==========
    def _on_joy(self, j: Joy):
        # 안전 인덱싱
        deadman = int(j.buttons[self.deadman_button]) if 0 <= self.deadman_button < len(j.buttons) else 0
        ret_cur = int(j.buttons[self.return_button]) if 0 <= self.return_button < len(j.buttons) else 0
        ret_prev = self._prev_btn.get(self.return_button, 0)
        self._prev_btn[self.return_button] = ret_cur
        return_rising = (ret_prev == 0 and ret_cur == 1)

        # ---- 우스틱 X축만 활동으로 인정 ----
        axis_active = False
        idx = self.right_x_axis
        if 0 <= idx < len(j.axes):
            try:
                val = float(j.axes[idx])
                axis_active = abs(val) > self.axis_deadzone
            except (ValueError, TypeError):
                axis_active = False
        else:
            # 축 인덱스가 잘못되면 활동 없음으로 처리하고 경고
            self.get_logger().warn(f'[enabled_guard] right_x_axis index {idx} out of range (len={len(j.axes)})')
            axis_active = False

        # ── 수동(조이스틱) 진입: deadman + (우스틱 X축 활동) ───────────────
        if deadman == 1 and axis_active and not self.manual_active:
            self.manual_active = True
            # 다른 노드 전부 OFF
            for n in self.targets:
                self._set_enabled(n, False)
            # 조이스틱 노드 ON
            self._set_enabled(self.joy_node, True)
            self.get_logger().info(f'[enabled_guard] MANUAL engaged -> joy_node ON, targets OFF (last_active={self.last_active})')
            return

        # ── 복귀: return 버튼 ─────────────────────────────────────────────
        if self.manual_active and return_rising:
            self.manual_active = False
            # 조이스틱 노드 OFF
            self._set_enabled(self.joy_node, False)
            # 마지막 자동 노드 ON
            target = self.last_active if self.last_active in self.targets else self.prefer
            self._set_enabled(target, True)
            self.get_logger().info(f'[enabled_guard] return -> joy_node OFF, {target}=True')

    # ========== 공용 유틸 ==========
    def _set_enabled(self, node_name: str, val: bool):
        cli = self.create_client(SetParameters, f'/{node_name}/set_parameters')
        if not cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().warn(f'set_parameters not available: {node_name}')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='drive_publish_enabled',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=val)
        )]
        cli.call_async(req)  # 비동기, 블로킹 없음

    def _get_enabled(self, node_name: str) -> bool:
        cli = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().warn(f'get_parameters not available: {node_name}')
            return False
        req = GetParameters.Request(); req.names = ['drive_publish_enabled']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.1)
        res = fut.result()
        if not res or not res.values:
            return False
        v = res.values[0]
        return v.type == ParameterType.PARAMETER_BOOL and bool(v.bool_value)

def main(args=None):
    rclpy.init(args=args)
    node = EnabledGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
