#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

from rcl_interfaces.msg import (
    SetParametersResult,
    Parameter as RclParamMsg,
    ParameterValue,
    ParameterType,
)
from rcl_interfaces.srv import GetParameters, SetParameters


class OfcJoyController(Node):
    def __init__(self):
        super().__init__('ofc_joy_controller')

        # -------- 파라미터 선언 --------
        self.declare_parameters('', [
            ('drive_topic', '/drive'),
            ('deadman_button', 4),     # PS5 L1
            ('axis_steer', 2),         # PS5 오른쪽 스틱 X축
            ('scale_steer', 0.34),     # 조향 각도 최대값 (rad)
            ('deadzone', 0.05),        # 조향 데드존
            ('fixed_speed', 2.5),      # 고정 속도 (m/s)
            ('publish_hz', 50.0),      # 발행 주기 (Hz)
            ('drive_publish_enabled', False),

            # ★ TLN 파라미터 증감용 설정
            ('tln_node_name', '/tln_inference'),
            ('tln_step', 0.5),         # v_min / v_max 증분 (m/s)
            ('vmin_floor', 0.0),       # v_min 하한
            ('vmax_ceil', 15.0),       # v_max 상한
        ])

        p = self.get_parameter
        self.drive_topic          = p('drive_topic').value
        self.deadman_button       = int(p('deadman_button').value)
        self.axis_steer           = int(p('axis_steer').value)
        self.scale_steer          = float(p('scale_steer').value)
        self.deadzone             = float(p('deadzone').value)
        self.fixed_speed          = float(p('fixed_speed').value)
        self.drive_publish_enabled = bool(p('drive_publish_enabled').value)

        # TLN 연동 파라미터
        self.tln_node_name = str(p('tln_node_name').value)
        self.tln_step      = float(p('tln_step').value)
        self.vmin_floor    = float(p('vmin_floor').value)
        self.vmax_ceil     = float(p('vmax_ceil').value)

        # -------- Pub/Sub --------
        self.sub_joy   = self.create_subscription(Joy, '/joy', self._on_joy, 50)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        # -------- 내부 상태 --------
        self._last_joy = None
        self._last_deadman = 0
        self.deadman_active = False
        self._last_pub_t = 0.0
        self._pub_dt = 1.0 / float(self.get_parameter('publish_hz').value)

        # 버튼 edge 검출용
        self._prev_buttons = []

        # TLN v_min / v_max 캐시
        self.tln_v_min = None
        self.tln_v_max = None

        # TLN parameter 서비스 클라이언트
        self.tln_get_cli = self.create_client(GetParameters, f'{self.tln_node_name}/get_parameters')
        self.tln_set_cli = self.create_client(SetParameters, f'{self.tln_node_name}/set_parameters')

        # 서비스 준비 대기 및 초기값 가져오기 (비차단)
        self.create_timer(0.1, self._try_init_tln_params)

        # 파라미터 변경 콜백 & 주기 발행 타이머
        self.add_on_set_parameters_callback(self._on_set_params)
        self.timer = self.create_timer(self._pub_dt, self._tick)

        self.get_logger().info('[ofc_joy_controller] ready (fixed speed + deadman + TLN v_min/v_max adjust)')

        # 버튼 매핑 (PS5 기준): △=3 / ×=1 / R3=9 / L3=8
        self.btn_vmax_up   = 3
        self.btn_vmax_down = 1
        self.btn_vmin_up   = 9
        self.btn_vmin_down = 8

    # ---------- 유틸: ParameterValue -> float ----------
    @staticmethod
    def _pv_to_float(pv: ParameterValue) -> float:
        if pv.type == ParameterType.PARAMETER_DOUBLE:
            return float(pv.double_value)
        if pv.type == ParameterType.PARAMETER_INTEGER:
            return float(pv.integer_value)
        # 다른 타입이면 NaN 반환 (호출부에서 무시/로그)
        return float('nan')

    # ---------- TLN 초기화 루틴 ----------
    def _try_init_tln_params(self):
        if self.tln_v_min is not None and self.tln_v_max is not None:
            return  # 이미 초기화됨

        ok_get = self.tln_get_cli.wait_for_service(timeout_sec=0.0)
        ok_set = self.tln_set_cli.wait_for_service(timeout_sec=0.0)
        if not (ok_get and ok_set):
            self.get_logger().debug(f'waiting TLN services: get={ok_get}, set={ok_set}')
            return

        # v_min / v_max 조회
        req = GetParameters.Request()
        req.names = ['v_min', 'v_max']
        future = self.tln_get_cli.call_async(req)

        def _done(_):
            res = future.result()
            if not res or len(res.values) < 2:
                self.get_logger().warn('[ofc_joy_controller] failed to get TLN v_min/v_max')
                return
            vmin = self._pv_to_float(res.values[0])
            vmax = self._pv_to_float(res.values[1])
            if vmin != vmin or vmax != vmax:  # NaN 체크
                self.get_logger().warn('[ofc_joy_controller] TLN v_min/v_max type not float/int')
                return
            self.tln_v_min = vmin
            self.tln_v_max = vmax
            self.get_logger().info(f'[ofc_joy_controller] TLN init v_min={self.tln_v_min:.2f}, v_max={self.tln_v_max:.2f}')

        future.add_done_callback(_done)

    # ---------- 파라미터 변경 콜백 ----------
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
            elif prm.name == 'tln_node_name':
                self.tln_node_name = str(prm.value)  # 런타임 교체는 재시작 권장
            elif prm.name == 'tln_step':
                self.tln_step = float(prm.value)
            elif prm.name == 'vmin_floor':
                self.vmin_floor = float(prm.value)
            elif prm.name == 'vmax_ceil':
                self.vmax_ceil = float(prm.value)
        return SetParametersResult(successful=True)

    # ---------- Joy 콜백 ----------
    def _on_joy(self, msg: Joy):
        self._last_joy = msg

        # deadman 처리
        deadman = self._btn(msg, self.deadman_button)

        # 버튼을 놓을 때 0,0 명령 발행(안전 정지)
        if self._last_deadman == 1 and deadman == 0 and self.drive_publish_enabled:
            self._publish(0.0, 0.0)

        self.deadman_active = (deadman == 1)
        self._last_deadman = deadman

        # --- TLN v_min / v_max 증감 (rising edge) ---
        if self.tln_v_min is not None and self.tln_v_max is not None:
            prev = self._prev_buttons or [0] * len(msg.buttons)
            # vmax up (△=3)
            if self._pressed_edge(prev, msg.buttons, self.btn_vmax_up):
                self._adjust_tln_v('v_max', +self.tln_step)
            # vmax down (×=1)
            if self._pressed_edge(prev, msg.buttons, self.btn_vmax_down):
                self._adjust_tln_v('v_max', -self.tln_step)
            # vmin up (R3=9)
            if self._pressed_edge(prev, msg.buttons, self.btn_vmin_up):
                self._adjust_tln_v('v_min', +self.tln_step)
            # vmin down (L3=8)
            if self._pressed_edge(prev, msg.buttons, self.btn_vmin_down):
                self._adjust_tln_v('v_min', -self.tln_step)

        self._prev_buttons = list(msg.buttons)

    # ---------- TLN 값 증감 & 전송 ----------
    def _adjust_tln_v(self, name: str, delta: float):
        # 로컬 캐시 기준 업데이트 + 경계/상호제약
        if name == 'v_min':
            new_vmin = max(self.vmin_floor, min(self.tln_v_min + delta, self.tln_v_max))
            self.tln_v_min = new_vmin
            value = new_vmin
        elif name == 'v_max':
            new_vmax = min(self.vmax_ceil, max(self.tln_v_max + delta, self.tln_v_min))
            self.tln_v_max = new_vmax
            value = new_vmax
        else:
            return

        # TLN 노드에 SetParameters 호출
        req = SetParameters.Request()
        prm = RclParamMsg()
        prm.name = name
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
        prm.value = pv
        req.parameters = [prm]

        fut = self.tln_set_cli.call_async(req)

        def _done(_):
            res = fut.result()
            ok = (res is not None and all(r.successful for r in res.results))
            self.get_logger().info(f'[ofc_joy_controller] set {name}={value:.2f} -> {"OK" if ok else "FAIL"}')

        fut.add_done_callback(_done)

    # ---------- 주기 발행 ----------
    def _tick(self):
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

    # ---------- 유틸 ----------
    def _map_steer(self, j: Joy) -> float:
        raw = self._axis(j, self.axis_steer)
        val = 0.0 if abs(raw) < self.deadzone else raw
        val = max(-1.0, min(1.0, val)) * self.scale_steer
        return float(val)

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

    @staticmethod
    def _pressed_edge(prev_buttons, cur_buttons, idx: int) -> bool:
        prev = prev_buttons[idx] if 0 <= idx < len(prev_buttons) else 0
        cur  = cur_buttons[idx] if 0 <= idx < len(cur_buttons) else 0
        return prev == 0 and cur == 1


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
