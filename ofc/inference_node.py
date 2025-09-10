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

# ===================== 기본 튜닝값 =====================
DOWN_SAMPLE = 2
EMA_ALPHA = 0.7
STEERING_DEADZONE_RAD = 0.0

CURVE_SPEED_WEIGHT = 0.65
V_MIN = 3.0
V_MAX = 6.5

STEER_ABS_MAX = 0.36
# =====================================================

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

            # 런타임 좌우반전 스위치(입력 뒤집고, 조향 부호 반전)
            ('reverse_direction', False),

            ('curve_speed_weight', CURVE_SPEED_WEIGHT),
            ('v_min', V_MIN),
            ('v_max', V_MAX),
            ('steer_abs_max', STEER_ABS_MAX),

            # 매핑 모드 스위치
            ('mapping', False),

            # 라이다 값 클램핑 상한 (기본 10.0 m)
            ('clamp_range_max_m', 10.0),
        ])

        # 값 로드
        self.drive_publish_enabled = bool(self.get_parameter("drive_publish_enabled").value)
        self.scan_topic            = str(self.get_parameter('scan_topic').value)
        self.drive_topic           = str(self.get_parameter('drive_topic').value)
        self.down_sample           = int(self.get_parameter('down_sample').value)
        self.ema_alpha             = float(self.get_parameter('ema_alpha').value)
        self.steering_deadzone_rad = float(self.get_parameter('steering_deadzone_rad').value)

        self.reverse_direction     = bool(self.get_parameter('reverse_direction').value)

        self.curve_speed_weight    = float(self.get_parameter('curve_speed_weight').value)
        self.v_min                 = float(self.get_parameter('v_min').value)
        self.v_max                 = float(self.get_parameter('v_max').value)
        self.steer_abs_max         = float(self.get_parameter('steer_abs_max').value)

        self.mapping               = bool(self.get_parameter('mapping').value)

        # 클램핑 상한 파라미터
        self.clamp_range_max_m     = float(self.get_parameter('clamp_range_max_m').value)

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
        # 1080 → 1081 보정(모델 입력 정합용)
        if arr.shape[0] == 1080:
            arr = np.append(arr, arr[-1])
        return arr

    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min

    def scan_cb(self, msg: LaserScan):
        ts = time.time()
        rng = np.asarray(msg.ranges, dtype=np.float64)
        rng = self.make_hokuyo_scan(rng)

        # 라이다 클램핑: NaN/±Inf 처리 후 [0, clamp_range_max_m]로 제한
        cap = float(self.clamp_range_max_m)
        if cap <= 0.0:
            cap = 10.0  # 방어적 기본값
        rng = np.nan_to_num(rng, nan=cap, posinf=cap, neginf=0.0)
        rng = np.clip(rng, 0.0, cap)

        # 역방향이면 입력 LiDAR 좌우 반전
        if self.reverse_direction:
            rng = rng[::-1].copy()

        arr = rng[::self.down_sample].reshape(1, -1, 1)
        steering_raw, speed_raw = self.dnn_output(arr)

        # 역방향이면 출력 조향만 부호 반전
        if self.reverse_direction:
            steering_raw = -steering_raw

        # ====== 보정(EMA/데드존/커브시 속도 감쇠/클립) ======
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

        # 매핑 모드일 때 속도만 1.0~3.0 m/s로 제한
        if self.mapping:
            if speed < 1.0: speed = 1.0
            if speed > 3.0: speed = 3.0
        # ===================================================

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
            elif p.name == 'mapping':
                self.mapping = bool(p.value)
                self.get_logger().info(f"[tln_inference] mapping = {self.mapping}")
            elif p.name == 'clamp_range_max_m':
                self.clamp_range_max_m = float(p.value)
                self.get_logger().info(f"[tln_inference] clamp_range_max_m = {self.clamp_range_max_m:.2f} m")
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

