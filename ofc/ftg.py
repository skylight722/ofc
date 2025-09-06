#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry  # ★ 추가

# ★ 추가: enabled 파라미터를 위한 import
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class FTG_Controller(Node):
    # Lidar processing params
    PREPROCESS_CONV_SIZE = 7

    # Steering params
    STRAIGHTS_STEERING_ANGLE = np.pi / 18   # 10 deg
    MILD_CURVE_ANGLE        = np.pi / 6     # 30 deg
    ULTRASTRAIGHTS_ANGLE    = np.pi / 60    # 3 deg

    def __init__(self):
        super().__init__('ftg_controller')

        # ── 파라미터 ─────────────────────────────────────────────────────
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mapping', False),
                ('debug', False),
                ('safety_radius', 4),
                ('max_lidar_dist', 10.0),
                ('max_speed', 8.0),
                ('range_offset', 75),
                ('track_width', 3.5),

                # ★ 추가: 속도/토픽 관련
                ('use_odom_velocity', True),
                ('odom_topic', '/car_state/odom'),
                ('vel_ema_alpha', 0.3),

                # ★ 추가: 토픽 통일(컨트롤러 매니저와 호환)
                ('scan_topic', '/scan'),
                ('drive_topic', '/drive'),

                # ★ 추가: 퍼블리시 on/off 제어
                ('drive_publish_enabled', False),
            ],
        )
        self.mapping        = self.get_parameter('mapping').value
        self.DEBUG          = self.get_parameter('debug').value
        self.SAFETY_RADIUS  = int(self.get_parameter('safety_radius').value)
        self.MAX_LIDAR_DIST = float(self.get_parameter('max_lidar_dist').value)
        self.MAX_SPEED      = float(self.get_parameter('max_speed').value)
        self.range_offset   = int(self.get_parameter('range_offset').value)
        self.track_width    = float(self.get_parameter('track_width').value)

        self.use_odom_velocity = bool(self.get_parameter('use_odom_velocity').value)
        self.odom_topic        = str(self.get_parameter('odom_topic').value)
        self.vel_ema_alpha     = float(self.get_parameter('vel_ema_alpha').value)
        self.scan_topic        = str(self.get_parameter('scan_topic').value)
        self.drive_topic       = str(self.get_parameter('drive_topic').value)

        # 속도 스케일
        scale = 0.6
        self.CORNERS_SPEED        = 0.3  * self.MAX_SPEED * scale
        self.MILD_CORNERS_SPEED   = 0.45 * self.MAX_SPEED * scale
        self.STRAIGHTS_SPEED      = 0.8  * self.MAX_SPEED * scale
        self.ULTRASTRAIGHTS_SPEED = 1.0  * self.MAX_SPEED * scale

        # 상태
        self.velocity = 0.0
        self.scan     = None
        self.radians_per_elem = None

        # ── 구독/퍼블리시 ───────────────────────────────────────────────
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, 10
        )
        self.pub_drive = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )

        self.scan_pub = self.create_publisher(MarkerArray, '/scan_proc/markers', 10)
        self.best_gap = self.create_publisher(MarkerArray, '/best_gap/markers', 10)
        self.best_pnt = self.create_publisher(Marker, '/best_points/marker', 10)

        # ★ 오도메 속도 구독(선택)
        if self.use_odom_velocity:
            self.sub_odom = self.create_subscription(
                Odometry, self.odom_topic, self.odom_cb, 10
            )

        # ★ 추가: enabled 파라미터 현재값과 변경 콜백
        self.drive_publish_enabled = bool(self.get_parameter('drive_publish_enabled').value)
        self.add_on_set_parameters_callback(self._on_params)

    # ★ enabled 외 파라미터도 런타임 반영
    def _on_params(self, params):
        ok = True
        for p in params:
            try:
                if p.name == 'drive_publish_enabled':
                    if p.type_ == Parameter.Type.BOOL:
                        self.drive_publish_enabled = bool(p.value)
                        self.get_logger().info(f"[ftg] drive_publish_enabled = {self.drive_publish_enabled}")
                    else:
                        self.get_logger().warn("enabled must be bool")
                        ok = False

                elif p.name == 'mapping':
                    self.mapping = bool(p.value)

                elif p.name == 'debug':
                    self.DEBUG = bool(p.value)

                elif p.name == 'safety_radius':
                    self.SAFETY_RADIUS = int(p.value)

                elif p.name == 'max_lidar_dist':
                    self.MAX_LIDAR_DIST = float(p.value)

                elif p.name == 'max_speed':
                    self.MAX_SPEED = float(p.value)
                    # 속도 티어 의존값 재계산 (원래 로직 유지)
                    scale = 0.6
                    self.CORNERS_SPEED        = 0.3  * self.MAX_SPEED * scale
                    self.MILD_CORNERS_SPEED   = 0.45 * self.MAX_SPEED * scale
                    self.STRAIGHTS_SPEED      = 0.8  * self.MAX_SPEED * scale
                    self.ULTRASTRAIGHTS_SPEED = 1.0  * self.MAX_SPEED * scale

                elif p.name == 'range_offset':
                    self.range_offset = int(p.value)

                elif p.name == 'track_width':
                    self.track_width = float(p.value)

                elif p.name == 'use_odom_velocity':
                    new = bool(p.value)
                    if new and not self.use_odom_velocity:
                        # 오도메 구독 시작
                        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
                    elif not new and self.use_odom_velocity:
                        # 오도메 구독 해제
                        if hasattr(self, 'sub_odom') and self.sub_odom:
                            try:
                                self.destroy_subscription(self.sub_odom)
                            except Exception:
                                pass
                            self.sub_odom = None
                    self.use_odom_velocity = new

                elif p.name == 'odom_topic':
                    new_topic = str(p.value)
                    self.odom_topic = new_topic
                    if self.use_odom_velocity:
                        # 런타임 리바인드
                        if hasattr(self, 'sub_odom') and self.sub_odom:
                            try:
                                self.destroy_subscription(self.sub_odom)
                            except Exception:
                                pass
                        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

                elif p.name == 'vel_ema_alpha':
                    self.vel_ema_alpha = float(p.value)

                # scan_topic / drive_topic은 보통 재시작 적용 (원하면 런타임 리바인드도 가능)
                elif p.name == 'scan_topic':
                    self.scan_topic = str(p.value)
                elif p.name == 'drive_topic':
                    self.drive_topic = str(p.value)

            except Exception:
                ok = False

        return SetParametersResult(successful=ok)

    # ★ 오도메 콜백: 논리 그대로 — 속도 = twist.twist.linear.x
    def odom_cb(self, msg: Odometry):
        v_x = float(msg.twist.twist.linear.x)
        if not np.isfinite(v_x):
            return
        # 부드럽게(EMA)
        a = self.vel_ema_alpha
        self.set_vel(a * v_x + (1.0 - a) * self.velocity)

    # ─────────────────────────── FTG 로직 ────────────────────────────
    def _preprocess_lidar(self, ranges) -> np.ndarray:
        self.radians_per_elem = (1.5 * np.pi) / len(ranges)  # -135~+135°
        proc_ranges = np.array(ranges[self.range_offset:-self.range_offset])
        proc_ranges = np.convolve(
            proc_ranges,
            np.ones(self.PREPROCESS_CONV_SIZE) / self.PREPROCESS_CONV_SIZE,
            'valid'
        )
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges[::-1]  # 오른→왼 정렬

    def _get_steer_angle(self, point_x, point_y) -> float:
        steering_angle = math.atan2(point_y, point_x)
        return float(np.clip(steering_angle, -0.4, 0.4))

    def _get_best_range_point(self, proc_ranges) -> tuple:
        radius = self._get_radius()
        gap_left, gap_right = self._find_largest_gap(proc_ranges, radius)
        gap_left  += self.range_offset - 180
        gap_right += self.range_offset - 180
        gap_mid = int((gap_right + gap_left) / 2)

        best_y = np.cos(gap_mid * self.radians_per_elem) * radius
        best_x = np.sin(gap_mid * self.radians_per_elem) * radius

        if self.DEBUG:
            self._delete_gap_markers()
            gap_markers = MarkerArray()
            for i in range(gap_left, gap_right):
                mrk = Marker()
                mrk.header.frame_id = 'car_state/laser'
                mrk.header.stamp = self.get_clock().now().to_msg()
                mrk.type = mrk.SPHERE
                mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.05
                mrk.color.a = 1.0; mrk.color.r = 1.0; mrk.color.g = 1.0
                mrk.id = int(i - gap_left)
                mrk.pose.position.y = math.cos(i * self.radians_per_elem) * radius
                mrk.pose.position.x = math.sin(i * self.radians_per_elem) * radius
                mrk.pose.orientation.w = 1.0
                gap_markers.markers.append(mrk)
            self.best_gap.publish(gap_markers)

            best_mrk = Marker()
            best_mrk.header.frame_id = 'car_state/laser'
            best_mrk.header.stamp = self.get_clock().now().to_msg()
            best_mrk.type = best_mrk.SPHERE
            best_mrk.scale.x = best_mrk.scale.y = best_mrk.scale.z = 0.2
            best_mrk.color.a = 1.0; best_mrk.color.g = 1.0; best_mrk.color.b = 1.0
            best_mrk.id = 0
            best_mrk.pose.position.y = float(best_y)
            best_mrk.pose.position.x = float(best_x)
            best_mrk.pose.orientation.w = 1.0
            self.best_pnt.publish(best_mrk)

        return float(best_x), float(best_y)

    def process_lidar(self, ranges) -> tuple:
        proc = self._preprocess_lidar(ranges)
        proc = self._safety_border(proc)

        if self.DEBUG:
            scan_markers = MarkerArray()
            for i, d in enumerate(proc):
                m = Marker()
                m.header.frame_id = 'car_state/laser'
                m.header.stamp = self.get_clock().now().to_msg()
                m.type = m.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 0.05
                m.color.a = 1.0; m.color.r = 1.0; m.color.b = 1.0
                m.id = i
                m.pose.position.x = math.sin(i * self.radians_per_elem) * d
                m.pose.position.y = math.cos(i * self.radians_per_elem) * d
                m.pose.orientation.w = 1.0
                scan_markers.markers.append(m)
            self.scan_pub.publish(scan_markers)

        bx, by = self._get_best_range_point(proc)
        steer  = self._get_steer_angle(bx, by)

        if self.mapping:
            speed = 1.5
        else:
            a = abs(steer)
            if a > self.MILD_CURVE_ANGLE:
                speed = self.CORNERS_SPEED
            elif a > self.STRAIGHTS_STEERING_ANGLE:
                speed = self.MILD_CORNERS_SPEED
            elif a > self.ULTRASTRAIGHTS_ANGLE:
                speed = self.STRAIGHTS_SPEED
            else:
                speed = self.ULTRASTRAIGHTS_SPEED

        return float(speed), float(steer)

    def _find_largest_gap(self, ranges, radius) -> tuple:
        bin_ranges = np.where(ranges >= radius, 1, 0)
        bin_diffs = np.abs(np.diff(bin_ranges))
        bin_diffs[0] = 1; bin_diffs[-1] = 1
        diff_idxs = bin_diffs.nonzero()[0]
        high_gaps = []
        for i in range(len(diff_idxs) - 1):
            low, high = diff_idxs[i], diff_idxs[i + 1]
            high_gaps.append(np.mean(bin_ranges[low:high]) > 0.5)
        gap_left  = diff_idxs[np.argmax(high_gaps * np.diff(diff_idxs))]
        gap_width = np.max(high_gaps * np.diff(diff_idxs))
        gap_right = gap_left + gap_width
        return int(gap_left), int(gap_right)

    def _get_radius(self) -> float:
        # ★ 오도메 속도 적용됨(EMA 반영 결과)
        return float(min(5.0, self.track_width / 2.0 + 2.0 * (self.velocity / self.MAX_SPEED)))

    def set_vel(self, velocity) -> None:
        self.velocity = float(velocity)

    def _safety_border(self, ranges) -> np.ndarray:
        filtered = list(ranges)
        n = len(ranges)
        i = 0
        while i < n - 1:
            if ranges[i + 1] - ranges[i] > 0.5:
                for j in range(self.SAFETY_RADIUS):
                    if i + j < n:
                        filtered[i + j] = ranges[i]
                i += self.SAFETY_RADIUS - 2
            i += 1
        i = n - 1
        while i > 0:
            if ranges[i - 1] - ranges[i] > 0.5:
                for j in range(self.SAFETY_RADIUS):
                    if i - j >= 0:
                        filtered[i - j] = ranges[i]
                i -= self.SAFETY_RADIUS - 2
            i -= 1
        return np.array(filtered, dtype=float)

    def _delete_gap_markers(self) -> None:
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'car_state/laser'
        m.header.stamp = self.get_clock().now().to_msg()
        m.action = m.DELETEALL
        m.id = 0
        arr.markers.append(m)
        self.best_gap.publish(arr)

    # ── 콜백 ───────────────────────────────────────────────────────────
    def scan_cb(self, msg: LaserScan):
        ts = time.time()
        speed, steer = self.process_lidar(msg.ranges)

        # 오도메 미사용 시, 퍼블리시한 속도를 내부 상태에 반영
        if not self.use_odom_velocity:
            a = self.vel_ema_alpha
            self.set_vel(a * float(speed) + (1.0 - a) * self.velocity)

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = float(speed)
        out.drive.steering_angle = float(steer)

        # ★ 핵심: enabled일 때만 퍼블리시
        if self.drive_publish_enabled:
            self.pub_drive.publish(out)
            print(f"Servo: {steer:.4f}, Speed: {speed:.3f} m/s | Took: {(time.time() - ts) * 1000:.2f} ms")


def main(args=None):
    rclpy.init(args=args)
    node = FTG_Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

