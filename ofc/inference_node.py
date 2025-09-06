#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import rclpy
import time
from rclpy.node import Node
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

# ===================== 기본 튜닝값(초기값) =====================
DOWN_SAMPLE = 2
EMA_ALPHA = 0.7
STEERING_DEADZONE_RAD = 0.0

# 우회전 보정 관련 파라미터는 선언만 유지(호환성), 실제 로직은 제거됨
ENABLE_RIGHT_TURN_FIX = False
DESIRED_LEFT_WALL_DISTANCE = 2.0
WALL_FOLLOW_STRENGTH = 0.15
SIDE_VIEW_DEG_MIN = 75
SIDE_VIEW_DEG_MAX = 105

CURVE_SPEED_WEIGHT = 0.65
V_MIN = 3.0
V_MAX = 6.5

STEER_ABS_MAX = 0.36
# =============================================================

class TLNInference(Node):
    def __init__(self):
        super().__init__('tln_inference')

        # 모델 로드
        pkg_share = get_package_share_directory('ofc')
        self.model_path = str(Path(pkg_share) / 'models' / 'f2_f4_silverstone_7lap.keras')
        self.model = tf.keras.models.load_model(self.model_path, compile=False, safe_mode=False)
        self.get_logger().info("모델 로드 완료 - GPU backend 활성 ")
        self.steer_ema = 0.0

        # ---------- 파라미터 선언 ----------
        self.declare_parameter("drive_publish_enabled", False)

        self.declare_parameters('', [
            ('scan_topic', '/scan'),
            ('drive_topic', '/drive'),

            ('down_sample', DOWN_SAMPLE),
            ('ema_alpha', EMA_ALPHA),
            ('steering_deadzone_rad', STEERING_DEADZONE_RAD),

            # ↓ 런타임 좌우반전 스위치(입력 뒤집고, 조향 부호 반전)
            ('reverse_direction', False),

            # 우회전 보정 파라미터(호환성 유지용; 로직은 제거)
            ('enable_right_turn_fix', ENABLE_RIGHT_TURN_FIX),
            ('desired_left_wall_distance', DESIRED_LEFT_WALL_DISTANCE),
            ('wall_follow_strength', WALL_FOLLOW_STRENGTH),
            ('side_view_deg_min', SIDE_VIEW_DEG_MIN),
            ('side_view_deg_max', SIDE_VIEW_DEG_MAX),

            ('curve_speed_weight', CURVE_SPEED_WEIGHT),
            ('v_min', V_MIN),
            ('v_max', V_MAX),
            ('steer_abs_max', STEER_ABS_MAX),

            # ★ 추가: 매핑 모드 스위치 (최소 수정)
            ('mapping', False),
        ])

        # 값 로드
        self.drive_publish_enabled       = bool(self.get_parameter("drive_publish_enabled").value)
        self.scan_topic                  = str(self.get_parameter('scan_topic').value)
        self.drive_topic                 = str(self.get_parameter('drive_topic').value)
        self.down_sample                 = int(self.get_parameter('down_sample').value)
        self.ema_alpha                   = float(self.get_parameter('ema_alpha').value)
        self.steering_deadzone_rad       = float(self.get_parameter('steering_deadzone_rad').value)

        self.reverse_direction           = bool(self.get_parameter('reverse_direction').value)

        # 아래 5개는 선언만 유지(로직 제거됨)
        self.enable_right_turn_fix       = bool(self.get_parameter('enable_right_turn_fix').value)
        self.desired_left_wall_distance  = float(self.get_parameter('desired_left_wall_distance').value)
        self.wall_follow_strength        = float(self.get_parameter('wall_follow_strength').value)
        self.side_view_deg_min           = int(self.get_parameter('side_view_deg_min').value)
        self.side_view_deg_max           = int(self.get_parameter('side_view_deg_max').value)

        self.curve_speed_weight          = float(self.get_parameter('curve_speed_weight').value)
        self.v_min                       = float(self.get_parameter('v_min').value)
        self.v_max                       = float(self.get_parameter('v_max').value)
        self.steer_abs_max               = float(self.get_parameter('steer_abs_max').value)

        # ★ 추가: 매핑 모드 현재값 로드
        self.mapping                     = bool(self.get_parameter('mapping').value)

        # 변경 콜백 등록
        self.add_on_set_parameters_callback(self._on_params)
        # -----------------------------------

        # 구독/퍼블리셔
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 5)
        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 5)

    def dnn_output(self, arr):
        if arr is None:
            return 0., 0.
        return self.model(arr, training=False).numpy()[0]

    def make_hokuyo_scan(self, arr):
        if arr.shape[0] == 1080:
            arr = np.append(arr, arr[-1])
        return arr

    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min

    def scan_cb(self, msg: LaserScan):
        ts = time.time()
        rng = np.asarray(msg.ranges, dtype=np.float64)
        rng = self.make_hokuyo_scan(rng)

        # ★ 역방향이면 입력 LiDAR 좌우 반전
        if self.reverse_direction:
            rng = rng[::-1].copy()

        arr = rng[::self.down_sample].reshape(1, -1, 1)
        steering_raw, speed_raw = self.dnn_output(arr)

        # ★ 역방향이면 출력 조향만 부호 반전
        if self.reverse_direction:
            steering_raw = -steering_raw

        # ====== 보정(EMA/데드존/커브시 속도 감쇠/클립)은 기존 그대로 유지 ======
        steering = self.steer_ema = (1.0 - self.ema_alpha) * self.steer_ema + self.ema_alpha * steering_raw
        if abs(steering) < self.steering_deadzone_rad:
            steering = 0.0

        model_speed = self.linear_map(speed_raw, -1, 1, self.v_min, self.v_max)
        turn_factor = min(abs(steering) / self.steer_abs_max, 1.0)
        curve_damping_ratio = 1.0 - turn_factor
        final_speed = model_speed * curve_damping_ratio
        speed = (1.0 - self.curve_speed_weight) * model_speed + self.curve_speed_weight * final_speed

        steering = float(np.clip(steering, -self.steer_abs_max, self.steer_abs_max))
        speed    = float(np.clip(speed,    self.v_min,          self.v_max))

        # ★ 추가: 매핑 모드일 때 속도만 1.0~3.0 m/s로 안전 제한 (최소 수정)
        if self.mapping:
            if speed < 1.0: speed = 1.0
            if speed > 3.0: speed = 3.0
        # ===============================================================

        out = AckermannDriveStamped()
        out.header.stamp = msg.header.stamp
        out.drive.steering_angle = steering
        out.drive.speed = speed

        if self.drive_publish_enabled:
            self.pub.publish(out)
            print(f"Servo: {steering:.4f}, Speed: {speed:.3f} m/s | Took: {(time.time() - ts) * 1000:.2f} ms")

    # 파라미터 변경 콜백
    def _on_params(self, params):
        for p in params:
            if p.name == "drive_publish_enabled" and p.type_ == Parameter.Type.BOOL:
                self.drive_publish_enabled = bool(p.value)
            elif p.name == 'scan_topic':
                self.scan_topic = str(p.value)  # 런타임 교체는 재시작 권장
            elif p.name == 'drive_topic':
                self.drive_topic = str(p.value)
            elif p.name == 'down_sample':
                self.down_sample = int(p.value)
            elif p.name == 'ema_alpha':
                self.ema_alpha = float(p.value)
            elif p.name == 'steering_deadzone_rad':
                self.steering_deadzone_rad = float(p.value)
            elif p.name == 'reverse_direction':
                self.reverse_direction = bool(p.value)
            elif p.name == 'curve_speed_weight':
                self.curve_speed_weight = float(p.value)
            elif p.name == 'v_min':
                self.v_min = float(p.value)
            elif p.name == 'v_max':
                self.v_max = float(p.value)
            elif p.name == 'steer_abs_max':
                self.steer_abs_max = float(p.value)
            # ★ 추가: 매핑 모드 토글 지원 (최소 수정)
            elif p.name == 'mapping':
                self.mapping = bool(p.value)
                self.get_logger().info(f"[tln_inference] mapping = {self.mapping}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = TLNInference()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

