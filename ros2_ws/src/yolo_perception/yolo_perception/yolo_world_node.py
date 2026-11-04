import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloWorldNode(Node):
    def __init__(self):
        super().__init__('yolo_world_node')

        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('classes_file', '/home/embedded/rover_nav/config/detection_classes.yaml')
        self.declare_parameter('min_confidence_to_report', 0.75)
        self.min_confidence = self.get_parameter('min_confidence_to_report').value

        self.conf_thresh = self.get_parameter('confidence_threshold').value
        classes_file = self.get_parameter('classes_file').value
        self.get_logger().info('YOLO-World 모델 로딩 중...')
        self.model = YOLO('yolov8s-world.pt')
        self.model.to('cuda')

        # YAML 파일에서 기본 탐지 클래스 로드
        self.default_classes = self.load_classes(classes_file)
        self.target_classes = self.default_classes
        self.model.set_classes(self.target_classes)

        self.get_logger().info(
            f'기본 탐지 클래스 {len(self.default_classes)}개 로드됨: '
            f'{classes_file}')

        self.bridge = CvBridge()
        self.latest_scan = None
        self.recent_distances = []
        self.tracked_objects = {}

        # 카메라 내부 파라미터 (Intrinsic Calibration 결과)
        self.FX = 1373.18756
        self.CX = 633.49902

        # 카메라-라이다 오프셋
        self.CAMERA_OFFSET_X = 0.0801   # 기존 0.0 → 0.0801
        self.CAMERA_OFFSET_Y = -0.0084  # 새로 추가
        self.THETA_OFFSET_DEG = -1.51   # 새로 추가
        self.MIN_PLAUSIBLE_DISTANCE = 0.3

        #self.subscription = self.create_subscription(
         #   Image, '/image_raw', self.image_callback, 10)

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, image_qos)

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, lidar_qos)

        # 텍스트 명령 구독 (탐색 대상 변경용)
        self.command_subscription = self.create_subscription(
            String, '/search_target', self.command_callback, 10)

        # 탐색 결과 발행 (Nav2 goal 계산에 활용 가능)
        self.result_publisher = self.create_publisher(
            String, '/perception_result', 10)

        self.get_logger().info('YOLO-World 노드 시작됨 (기본 탐색 대상: person)')
        self.get_logger().info('명령 예시: ros2 topic pub /search_target std_msgs/msg/String "data: \'red backpack\'"')

    def load_classes(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            classes = data.get('default_classes', [])
            if not classes:
                self.get_logger().warn(
                    f'{path}에서 클래스 목록을 찾지 못해 기본값(person)만 사용합니다.')
                return ['person']
            return classes
        except Exception as e:
            self.get_logger().error(
                f'클래스 파일 로드 실패({e}), 기본값(person)만 사용합니다.')
            return ['person']

    def command_callback(self, msg: String):
        new_target = msg.data.strip()
        
        if new_target == 'ALL':
            # 탐사 모드: 기본 다중 클래스 목록으로 복귀
            self.target_classes = self.default_classes
            self.model.set_classes(self.target_classes)
            self.get_logger().info(f'탐사 모드: {len(self.target_classes)}개 클래스 동시 탐지')
        elif ',' in new_target:
            # 콤마로 여러 개 직접 지정도 가능
            self.target_classes = [c.strip() for c in new_target.split(',')]
            self.model.set_classes(self.target_classes)
            self.get_logger().info(f'다중 탐색 대상 변경: {self.target_classes}')
        else:
            # 미션 모드: 단일 물체 정밀 탐색
            self.target_classes = [new_target]
            self.model.set_classes(self.target_classes)
            self.get_logger().info(f'단일 탐색 대상 변경: "{new_target}"')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def pixel_to_angle(self, center_x):
        x_normalized = (center_x - self.CX) / self.FX
        angle_rad = math.atan(x_normalized)
        return math.degrees(angle_rad)

    def get_distance_at_angle(self, angle_deg, search_window=8):
        if self.latest_scan is None:
            return None

        assumed_distance = 1.5
        cam_angle_rad = math.radians(angle_deg)
        x_cam = assumed_distance * math.cos(cam_angle_rad)
        y_cam = assumed_distance * math.sin(cam_angle_rad)

        x_lidar = x_cam + self.CAMERA_OFFSET_X
        y_lidar = y_cam + self.CAMERA_OFFSET_Y

        # atan2 결과는 "카메라 전방=0도" 기준의 각도
        angle_in_cam_frame = math.atan2(y_lidar, x_lidar) + math.radians(self.THETA_OFFSET_DEG)
        angle_in_cam_frame_deg = math.degrees(angle_in_cam_frame)

        lidar_raw_deg = 180.0 + angle_in_cam_frame_deg
        while lidar_raw_deg > 180.0:
            lidar_raw_deg -= 360.0
        while lidar_raw_deg < -180.0:
            lidar_raw_deg += 360.0
        
        lidar_raw_rad = math.radians(lidar_raw_deg)
        center_index = int((lidar_raw_rad - self.latest_scan.angle_min)
                            / self.latest_scan.angle_increment)

        ranges = self.latest_scan.ranges
        n = len(ranges)

        valid_distances = []
        for offset in range(-search_window, search_window + 1):
            idx = center_index + offset
            if 0 <= idx < n:
                d = ranges[idx]
                if d != 0.0 and d != float('inf') and not math.isnan(d) \
                   and d >= self.MIN_PLAUSIBLE_DISTANCE:
                    valid_distances.append(d)

        if not valid_distances:
            return None

        return min(valid_distances)

    def filter_distance(self, new_distance, max_history=5):
        self.recent_distances.append(new_distance)
        if len(self.recent_distances) > max_history:
            self.recent_distances.pop(0)
        sorted_d = sorted(self.recent_distances)
        return sorted_d[len(sorted_d) // 2]

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec
    
        results = self.model.track(
            frame, device='cuda', verbose=False,
            conf=self.conf_thresh,
            persist=True,
            tracker='bytetrack.yaml'
        )
        now = self.get_clock().now()
        current_ids = set()

        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            track_id = int(box.id[0]) if box.id is not None else -1

            # 여기서 먼저 걸러냄 — 기준 미달이면 아예 처리하지 않음
            if conf < self.min_confidence:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x = (x1 + x2) / 2
            angle_deg = self.pixel_to_angle(center_x)
            raw_distance = self.get_distance_at_angle(angle_deg)

            if raw_distance is not None:
                filtered_distance = self.filter_distance(raw_distance)
                # 트랙 상태 갱신 (마지막으로 확인된 위치/시각 기록)
                self.tracked_objects[track_id] = {
                    'cls_name': cls_name,
                    'angle': angle_deg,
                    'distance': filtered_distance,
                    'stamp_sec': stamp_sec,
                    'stamp_nanosec': stamp_nanosec,
                    'last_seen': now,
                }
                current_ids.add(track_id)

                result_msg = String()
                result_msg.data = (
                    f'{cls_name}|{angle_deg:.1f}|{filtered_distance:.2f}'
                    f'|{stamp_sec}|{stamp_nanosec}|{track_id}'
                )
                self.result_publisher.publish(result_msg)

                self.get_logger().info(
                    f'"{cls_name}"(id={track_id} 발견 (conf={conf:.2f}), '
                    f'각도={angle_deg:+.1f}도, 거리={filtered_distance:.2f}m')
        # 오래된(사라진) 트랙 정리 (예: 10초 이상 미확인 시 제거)
        stale_ids = [
            tid for tid, data in self.tracked_objects.items()
            if (now - data['last_seen']).nanoseconds / 1e9 > 10.0
        ]
        for tid in stale_ids:
            del self.tracked_objects[tid]


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
