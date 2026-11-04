import math
import csv
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge


class CalibrationCollector(Node):
    def __init__(self):
        super().__init__('calibration_collector')

        # 카메라 내부 파라미터 (기존 intrinsic calibration 결과 재사용)
        self.FX = 1373.18756
        self.CX = 633.49902

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_scan = None
        self.click_x = None  # 사용자가 클릭한 픽셀 x좌표

        self.collected_data = []  # (cam_angle, lidar_angle, lidar_distance)

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            Image, '/image_raw', self.image_callback, image_qos)

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, lidar_qos)

        cv2.namedWindow('Calibration')
        cv2.setMouseCallback('Calibration', self.mouse_callback)

        self.get_logger().info(
            '캘리브레이션 데이터 수집기 시작. '
            '이미지에서 상자 중심을 클릭 -> s 저장, q 종료')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_x = x

    def image_callback(self, msg: Image):
        self.latest_frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def pixel_to_angle(self, center_x):
        x_normalized = (center_x - self.CX) / self.FX
        angle_rad = math.atan(x_normalized)
        return math.degrees(angle_rad)

    def find_nearest_segment(self, target_angle_deg=None, angle_window_deg=30, max_range=4.0, cluster_gap=0.15):
        """가장 가까운 물체의 직선(클러스터) 세그먼트를 찾아 중심 각도/거리 반환"""
        if self.latest_scan is None:
            return None

        scan = self.latest_scan
        ranges = scan.ranges
        n = len(ranges)

        # 카메라 각도(target)를 라이다 각도 체계로 변환
        lidar_target = None
        if target_angle_deg is not None:
            lidar_target = 180.0 + target_angle_deg
            while lidar_target > 180.0:
                lidar_target -= 360.0
            while lidar_target < -180.0:
                lidar_target += 360.0

        points = []
        for i in range(n):
            r = ranges[i]
            if r == 0.0 or math.isnan(r) or r == float('inf') or r > max_range:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle)

            # wraparound 정규화만 적용 (보정 없음)
            while angle_deg > 180.0:
                angle_deg -= 360.0
            while angle_deg < -180.0:
                angle_deg += 360.0

            if lidar_target is not None:
                angle_diff = abs(angle_deg - lidar_target)
                angle_diff = min(angle_diff, 360 - angle_diff)
                if angle_diff > angle_window_deg:
                    continue

            points.append((i, math.radians(angle_deg), r))

        if not points:
            return None

        # 거리 기준으로 클러스터링 (인접한 점들의 거리 차이가 작으면 같은 클러스터)
        clusters = []
        current_cluster = [points[0]]
        for prev, curr in zip(points, points[1:]):
            if abs(curr[2] - prev[2]) < cluster_gap and (curr[0] - prev[0]) <= 3:
                current_cluster.append(curr)
            else:
                clusters.append(current_cluster)
                current_cluster = [curr]
        clusters.append(current_cluster)

        # 가장 가까운(평균 거리가 최소인) 클러스터 선택
        best_cluster = min(
            clusters, key=lambda c: sum(p[2] for p in c) / len(c))

        if len(best_cluster) < 3:
            return None  # 너무 작은 클러스터는 노이즈로 간주

        avg_angle_lidar = sum(p[1] for p in best_cluster) / len(best_cluster)
        avg_dist = sum(p[2] for p in best_cluster) / len(best_cluster)

        # 반환 시 다시 카메라 좌표계로 변환해서 일관성 유지
        avg_angle_lidar_deg = math.degrees(avg_angle_lidar)
        avg_angle_cam_frame = avg_angle_lidar_deg - 180.0
        while avg_angle_cam_frame > 180.0:
            avg_angle_cam_frame -= 360.0
        while avg_angle_cam_frame < -180.0:
            avg_angle_cam_frame += 360.0

        return avg_angle_cam_frame, avg_dist, len(best_cluster)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.latest_frame is None:
                continue

            display = self.latest_frame.copy()
            h, w = display.shape[:2]
            cam_angle = None
            segment = None

            # 클릭한 위치 표시
            if self.click_x is not None:
                cam_angle = self.pixel_to_angle(self.click_x)
                segment = self.find_nearest_segment(
                    target_angle_deg=cam_angle, angle_window_deg=20)
        
                cv2.line(display, (self.click_x, 0),
                         (self.click_x, h), (0, 255, 0), 2)
                cv2.putText(
                    display, f'Cam angle: {cam_angle:.1f} deg',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2)

            # 라이다에서 검출된 세그먼트 정보 표시
            if segment is not None:
                lidar_angle, lidar_dist, n_points = segment
                cv2.putText(
                    display,
                    f'Lidar: angle={lidar_angle:.1f} deg, '
                    f'dist={lidar_dist:.2f}m (n={n_points})',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 200, 255), 2)

            else:
                cv2.putText(
                    display, 'Lidar: no segment found near cam angle',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)

            cv2.putText(
                display, f'Collected: {len(self.collected_data)} points',
                (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (255, 255, 255), 2)

            cv2.imshow('Calibration', display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('s'):
                if self.click_x is not None and segment is not None:
                    lidar_angle, lidar_dist, _ = segment
                    self.collected_data.append(
                        (cam_angle, lidar_angle, lidar_dist))
                    self.get_logger().info(
                        f'저장됨 #{len(self.collected_data)}: '
                        f'cam={cam_angle:.1f}, lidar={lidar_angle:.1f}, '
                        f'dist={lidar_dist:.2f}')
                    self.click_x = None
                else:
                    self.get_logger().warn(
                        '클릭 또는 라이다 세그먼트가 없어 저장 실패')

            elif key == ord('q'):
                break

        self.save_to_csv()
        cv2.destroyAllWindows()

    def save_to_csv(self):
        path = '/home/embedded/rover_nav/config/calibration_data.csv'
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['cam_angle_deg', 'lidar_angle_deg', 'lidar_distance_m'])
            writer.writerows(self.collected_data)
        self.get_logger().info(f'{len(self.collected_data)}개 데이터를 {path}에 저장')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCollector()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
