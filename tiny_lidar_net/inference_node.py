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

# ===================== 튜닝 파라미터 (우회전 보정 최종) =====================
DOWN_SAMPLE = 2
EMA_ALPHA = 0.2
STEERING_DEADZONE_RAD = 0.005

# 우회전 진입 라인 보정 로직
# 이 기능을 켤지 끌지 결정
ENABLE_RIGHT_TURN_FIX = True
# 우회전 진입 시, 왼쪽 벽과 이 거리(미터)를 유지하도록 유도
DESIRED_LEFT_WALL_DISTANCE = 2.0
# 얼마나 강하게 왼쪽으로 붙일지 결정 (값이 클수록 더 강하게 붙음)
WALL_FOLLOW_STRENGTH = 0.15
# 측면을 감지할 각도 범위
SIDE_VIEW_DEG_MIN = 75
SIDE_VIEW_DEG_MAX = 105

# 속도 제어
CURVE_SPEED_WEIGHT = 0.65
V_MIN = 2.0
V_MAX = 10.0

# 조향각 제한
STEER_ABS_MAX = 0.25
# ======================================================================

class TLNInference(Node):
    def __init__(self):
        super().__init__('tln_inference')
        pkg_share = get_package_share_directory('tiny_lidar_net')
        self.model_path = str(Path(pkg_share) / 'models' / 'f2_f4_silverstone_7lap.keras')
        self.model = tf.keras.models.load_model(self.model_path, compile=False, safe_mode=False)
        self.get_logger().info("모델 로드 완료 - GPU backend 활성 ")
        self.steer_ema = 0.0
        self.sub = self.create_subscription(LaserScan, '/sim/scan', self.scan_cb, 5)
        self.pub = self.create_publisher(AckermannDriveStamped, '/sim/drive', 5)

    def dnn_output(self, arr):
        if arr is None: return 0., 0.
        return self.model(arr, training=False).numpy()[0]

    def make_hokuyo_scan(self, arr):
        if(arr.shape[0] == 1080): arr = np.append(arr, arr[-1])
        return arr

    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min

    def scan_cb(self, msg: LaserScan):
        ts = time.time()
        rng = np.asarray(msg.ranges, dtype=np.float64)
        rng = self.make_hokuyo_scan(rng)
        arr = rng[::DOWN_SAMPLE].reshape(1, -1, 1)

        steering_raw, speed_raw = self.dnn_output(arr)

        # 우회전 진입 라인 보정 로직
        if ENABLE_RIGHT_TURN_FIX and steering_raw < -0.05: # 모델이 우회전을 의도할 때
            # 1. 왼쪽 벽과의 거리 측정
            def angle_to_index(angle_deg):
                return int((np.deg2rad(angle_deg) - msg.angle_min) / msg.angle_increment)
            
            start_idx = angle_to_index(SIDE_VIEW_DEG_MIN)
            end_idx = angle_to_index(SIDE_VIEW_DEG_MAX)
            side_beams = rng[start_idx:end_idx]
            valid_beams = side_beams[np.isfinite(side_beams)]
            
            if len(valid_beams) > 0:
                left_distance = np.mean(valid_beams)
                
                # 2. 목표 거리와의 오차 계산
                error = DESIRED_LEFT_WALL_DISTANCE - left_distance
                
                # 3. 오차에 기반한 보정 조향값 계산 (왼쪽으로 더 붙으라는 명령)
                #    차가 왼쪽 벽에서 너무 멀리 떨어져 있을수록(error가 음수일수록) 
                #    왼쪽으로 꺾으라는(+) 보정값이 강하게 들어감
                correction_steer = -error * WALL_FOLLOW_STRENGTH
                
                # 4. 모델의 예측에 보정값을 더해줌
                steering_raw += correction_steer
        # ============================================

        steering = self.steer_ema = (1.0 - EMA_ALPHA) * self.steer_ema + EMA_ALPHA * steering_raw
        if abs(steering) < STEERING_DEADZONE_RAD:
            steering = 0.0

        model_speed = self.linear_map(speed_raw, -1, 1, V_MIN, V_MAX)
        turn_factor = min(abs(steering) / STEER_ABS_MAX, 1.0)
        curve_damping_ratio = 1.0 - turn_factor
        final_speed = model_speed * curve_damping_ratio
        speed = (1.0 - CURVE_SPEED_WEIGHT) * model_speed + CURVE_SPEED_WEIGHT * final_speed

        steering = float(np.clip(steering, -STEER_ABS_MAX, STEER_ABS_MAX))
        speed = float(np.clip(speed, V_MIN, V_MAX))

        out = AckermannDriveStamped()
        out.header.stamp = msg.header.stamp
        out.drive.steering_angle = steering
        out.drive.speed = speed
        self.pub.publish(out)

        print(f"Servo: {steering:.4f}, Speed: {speed:.3f} m/s | Took: {(time.time() - ts) * 1000:.2f} ms")


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
