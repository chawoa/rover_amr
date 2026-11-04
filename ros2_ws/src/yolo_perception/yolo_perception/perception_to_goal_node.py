import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 (PointStamped 변환 등록용)


class PerceptionToGoalNode(Node):
    def __init__(self):
        super().__init__('perception_to_goal_node')

        self.declare_parameter('approach_distance', 0.5)  # 물체 앞 0.5m에서 정지
        self.approach_distance = self.get_parameter('approach_distance').value

        self.latest_result = None  # (class_name, angle_deg, distance_m)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.perception_sub = self.create_subscription(
            String, '/perception_result', self.perception_callback, 10)

        self.command_sub = self.create_subscription(
            String, '/go_to_target', self.command_callback, 10)

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        self.get_logger().info('Perception-to-Goal 노드 시작됨')
        self.get_logger().info(
            '명령 예시: ros2 topic pub /go_to_target std_msgs/msg/String "data: \'phone\'" --once')

    def perception_callback(self, msg: String):
        try:
            cls_name, angle_str, dist_str = msg.data.split('|')
            self.latest_result = (cls_name, float(angle_str), float(dist_str))
        except ValueError:
            self.get_logger().warn(f'파싱 실패: {msg.data}')

    def command_callback(self, msg: String):
        target_class = msg.data.strip()

        if self.latest_result is None:
            self.get_logger().warn('아직 감지된 물체가 없습니다.')
            return

        cls_name, angle_deg, distance = self.latest_result

        if cls_name != target_class:
            self.get_logger().warn(
                f'현재 감지된 물체("{cls_name}")가 '
                f'요청한 대상("{target_class}")과 다릅니다.')
            return

        self.send_goal(angle_deg, distance)

    def send_goal(self, angle_deg, distance):
        # 물체 바로 앞이 아니라 approach_distance만큼 앞에서 정지
        target_distance = max(distance - self.approach_distance, 0.3)

        angle_rad = math.radians(angle_deg)

        # base_link 기준 좌표 (전방 +x, 좌측 +y 가정)
        x_base = target_distance * math.cos(angle_rad)
        y_base = target_distance * math.sin(angle_rad)

        point_base = PointStamped()
        point_base.header.frame_id = 'base_link'
        point_base.header.stamp = self.get_clock().now().to_msg()
        point_base.point.x = x_base
        point_base.point.y = y_base
        point_base.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f'TF 변환 실패: {e}')
            return

        point_map = tf2_geometry_msgs.do_transform_point(point_base, transform)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = point_map.point.x
        goal.pose.position.y = point_map.point.y
        goal.pose.position.z = 0.0

        # 물체를 향해 바라보도록 방향 설정 (로봇 현재 방향 기준 각도 반영)
        yaw = math.atan2(y_base, x_base) + self._current_yaw()
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'목표 전송: map 기준 ({point_map.point.x:.2f}, '
            f'{point_map.point.y:.2f}), 대상까지 원래 거리={distance:.2f}m')

    def _current_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
