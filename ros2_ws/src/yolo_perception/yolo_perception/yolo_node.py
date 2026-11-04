import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

FX = 1373.18756
CX = 633.49902
MIN_PLAUSIBLE_DISTANCE = 0.3

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('target_classes', ['person'])
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('hfov_deg', 38.6)

        model_path = self.get_parameter('model_path').value
        self.target_classes = self.get_parameter('target_classes').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.hfov_deg = self.get_parameter('hfov_deg').value

        self.get_logger().info(f'YOLO 모델 로딩: {model_path}')
        self.model = YOLO(model_path)
        self.model.to('cuda')

        self.bridge = CvBridge()
        self.latest_scan = None
        self.recent_distances = []  # 추가

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, lidar_qos)

        self.get_logger().info('YOLO Perception 노드 시작됨')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_distance_at_angle(self, angle_deg, search_window=15):
        CAMERA_OFFSET_X = 0.16
        if self.latest_scan is None:
            return None

        assumed_distance = 1.5  # 실내 평균 거리로 가정

        cam_angle_rad = math.radians(angle_deg)
        x_cam = assumed_distance * math.cos(cam_angle_rad)
        y_cam = assumed_distance * math.sin(cam_angle_rad)

        x_lidar = x_cam + CAMERA_OFFSET_X
        y_lidar = y_cam

        lidar_angle_rad = math.atan2(y_lidar, x_lidar)
        corrected_angle_rad = lidar_angle_rad + math.pi

        while corrected_angle_rad > math.pi:
            corrected_angle_rad -= 2 * math.pi
        while corrected_angle_rad < -math.pi:
            corrected_angle_rad += 2 * math.pi

        center_index = int((corrected_angle_rad - self.latest_scan.angle_min)
                            / self.latest_scan.angle_increment)

        ranges = self.latest_scan.ranges
        n = len(ranges)

        valid_distances = []
        for offset in range(-search_window, search_window + 1):
            idx = center_index + offset
            if 0 <= idx < n:
                d = ranges[idx]
                if d != 0.0 and d != float('inf') and not math.isnan(d) and d >= MIN_PLAUSIBLE_DISTANCE:
                    valid_distances.append(d)

        if not valid_distances:
            return None
        return min(valid_distances)

    def pixel_to_angle(self, center_x):
        x_normalized = (center_x - CX) / FX
        angle_rad = math.atan(x_normalized)
        return math.degrees(angle_rad)

    def filter_distance(self, new_distance, max_history=5):
        """최근 N개 거리값의 중앙값을 반환해서 순간적인 이상치를 걸러냄"""
        self.recent_distances.append(new_distance)
        if len(self.recent_distances) > max_history:
            self.recent_distances.pop(0)

        sorted_d = sorted(self.recent_distances)
        return sorted_d[len(sorted_d) // 2]

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_width = frame.shape[1]

        results = self.model(frame, device='cuda', verbose=False,
                              conf=self.conf_thresh)

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            cls_name = self.model.names[cls_id]

            if cls_name not in self.target_classes:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x = (x1 + x2) / 2

            # normalized_offset = (center_x - image_width / 2) / (image_width / 2)
            # angle_deg = normalized_offset * (self.hfov_deg / 2)
            angle_deg = self.pixel_to_angle(center_x)
            raw_distance = self.get_distance_at_angle(angle_deg)

            if raw_distance is not None:
                filtered_distance = self.filter_distance(raw_distance)
                self.get_logger().info(
                    f'{cls_name} 감지 (conf={conf:.2f}), '
                    f'각도={angle_deg:+.1f}도, 거리={filtered_distance:.2f}m')
            else:
                self.get_logger().info(
                    f'{cls_name} 감지 (conf={conf:.2f}), '
                    f'각도={angle_deg:+.1f}도, 거리=측정불가')


def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
