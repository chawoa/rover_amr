import json
import re
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PointStamped
import math
import numpy as np
from sklearn.cluster import DBSCAN
import ollama


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__('semantic_map_node')

        self.declare_parameter('model_name', 'qwen2.5:3b')
        self.declare_parameter('cluster_eps', 1.5)       # 군집 반경 (m)
        self.declare_parameter('cluster_min_samples', 2)  # 최소 물체 수
        self.declare_parameter('save_path',
                                '/home/embedded/rover_nav/config/semantic_map.json')

        self.model_name = self.get_parameter('model_name').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.cluster_min_samples = self.get_parameter('cluster_min_samples').value
        self.save_path = self.get_parameter('save_path').value

        # 발견한 물체 저장: [(class_name, map_x, map_y, timestamp), ...]
        self.observed_objects = []
        self.semantic_map = {}  # {cluster_id: {"room_type":..., "center":..., "objects":[...]}}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # YOLO-World 결과 구독 (Phase 2에서 만든 형식: "class|angle|distance")
        self.create_subscription(
            String, '/perception_result', self.perception_callback, 10)

        # 수동 트리거: 지금까지 모은 물체로 방 라벨링 실행
        self.create_subscription(
            String, '/build_semantic_map', self.build_map_callback, 10)

        self.query_pub = self.create_publisher(String, '/semantic_map_result', 10)

        self.get_logger().info('Semantic Map 노드 시작됨')
        self.get_logger().info(
            '탐사 중 감지된 물체가 자동 누적됩니다. '
            '방 라벨링 실행: ros2 topic pub /build_semantic_map std_msgs/msg/String "data: \'go\'" --once')

    # ---------------- 물체 수집 ----------------

    def perception_callback(self, msg: String):
        try:
            cls_name, angle_str, dist_str = msg.data.split('|')
            angle_deg = float(angle_str)
            distance = float(dist_str)
        except ValueError:
            return

        map_point = self.to_map_coords(angle_deg, distance)
        if map_point is None:
            return

        x, y = map_point
        now = time.time()

        # 이미 아주 가까운 위치(0.5m 이내)에 같은 클래스가 있으면 중복 저장 방지
        for obj in self.observed_objects:
            if obj[0] == cls_name:
                dx = obj[1] - x
                dy = obj[2] - y
                if math.hypot(dx, dy) < 0.5:
                    return

        self.observed_objects.append((cls_name, x, y, now))
        self.get_logger().info(
            f'물체 기록: {cls_name} @ ({x:.2f}, {y:.2f}) '
            f'[총 {len(self.observed_objects)}개 누적]')

    def to_map_coords(self, angle_deg, distance):
        angle_rad = math.radians(angle_deg)
        x_base = distance * math.cos(angle_rad)
        y_base = distance * math.sin(angle_rad)

        point_base = PointStamped()
        point_base.header.frame_id = 'base_link'
        point_base.header.stamp = self.get_clock().now().to_msg()
        point_base.point.x = x_base
        point_base.point.y = y_base

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception:
            return None

        point_map = tf2_geometry_msgs.do_transform_point(point_base, transform)
        return (point_map.point.x, point_map.point.y)

    # ---------------- 군집화 + LLM 방 라벨링 ----------------

    def build_map_callback(self, msg: String):
        if len(self.observed_objects) < self.cluster_min_samples:
            self.get_logger().warn(
                f'누적된 물체가 너무 적습니다 ({len(self.observed_objects)}개). '
                f'더 탐사한 뒤 다시 시도해 주세요.')
            return

        self.get_logger().info(
            f'{len(self.observed_objects)}개 물체로 방 라벨링을 시작합니다...')

        coords = np.array([[obj[1], obj[2]] for obj in self.observed_objects])
        clustering = DBSCAN(
            eps=self.cluster_eps,
            min_samples=self.cluster_min_samples
        ).fit(coords)

        labels = clustering.labels_
        unique_clusters = set(labels) - {-1}  # -1은 노이즈(군집 안 됨)

        self.get_logger().info(
            f'{len(unique_clusters)}개의 공간 군집을 발견했습니다.')

        self.semantic_map = {}

        for cluster_id in unique_clusters:
            indices = [i for i, l in enumerate(labels) if l == cluster_id]
            cluster_objects = [self.observed_objects[i] for i in indices]

            object_names = list({obj[0] for obj in cluster_objects})
            center_x = float(np.mean([obj[1] for obj in cluster_objects]))
            center_y = float(np.mean([obj[2] for obj in cluster_objects]))

            room_type = self.infer_room_type(object_names)

            self.semantic_map[str(cluster_id)] = {
                'room_type': room_type,
                'center': [center_x, center_y],
                'objects': object_names
            }

            self.get_logger().info(
                f'군집 {cluster_id}: {object_names} → "{room_type}" '
                f'(중심: {center_x:.2f}, {center_y:.2f})')

        self.save_semantic_map()

        result_msg = String()
        result_msg.data = json.dumps(self.semantic_map, ensure_ascii=False)
        self.query_pub.publish(result_msg)

    def infer_room_type(self, object_names):
        candidate_rooms = [
            "kitchen", "living room", "bedroom", "bathroom",
            "office", "dining room", "hallway", "storage room"
        ]

        prompt = (
            f"이 공간에 있는 물체 목록은 다음과 같다: {', '.join(object_names)}. "
            f"다음 후보 중 가장 어울리는 방 종류를 하나만 골라줘: "
            f"{', '.join(candidate_rooms)}. "
            f"반드시 JSON만 출력해: {{\"room_type\": \"방종류\"}}"
        )

        try:
            response = ollama.chat(
                model=self.model_name,
                messages=[{'role': 'user', 'content': prompt}]
            )
            reply_text = response['message']['content'].strip()
            match = re.search(r'\{[^}]*\}', reply_text)
            if not match:
                return "unknown"
            parsed = json.loads(match.group(0))
            return parsed.get('room_type', 'unknown')
        except Exception as e:
            self.get_logger().error(f'방 유형 추론 실패: {e}')
            return "unknown"

    def save_semantic_map(self):
        with open(self.save_path, 'w') as f:
            json.dump(self.semantic_map, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f'Semantic Map 저장됨: {self.save_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
