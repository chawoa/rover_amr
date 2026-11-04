import math
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import Spin, NavigateToPose
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class SupplementDeliveryNode(Node):
    def __init__(self):
        super().__init__('supplement_delivery_node')

        self.declare_parameter('approach_distance', 0.6)
        self.declare_parameter('search_timeout_sec', 30.0)
        self.declare_parameter('lost_timeout_sec', 5.0)
        self.declare_parameter('dispense_timeout_sec', 15.0)

        self.approach_distance = self.get_parameter('approach_distance').value
        self.search_timeout = self.get_parameter('search_timeout_sec').value
        self.lost_timeout = self.get_parameter('lost_timeout_sec').value
        self.dispense_timeout = self.get_parameter('dispense_timeout_sec').value

        # 상태: IDLE, SEARCHING, MOVING, DISPENSING, DONE, FAILED
        self.state = 'IDLE'
        self.current_mission = None   # schedule_manager로부터 받은 dict
        self.latest_result = None     # (class_name, angle_deg, distance_m)
        self.last_seen_time = 0.0
        self.state_start_time = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # schedule_manager로부터 미션 수신
        self.create_subscription(
            String, '/supplement_mission', self.mission_callback, 10)

        # perception 결과 수신 (기존 yolo_world_node 결과 재사용)
        self.create_subscription(
            String, '/perception_result', self.perception_callback, 10)

        # yolo_world_node에게 탐색 대상 지정
        self.search_pub = self.create_publisher(String, '/search_target', 10)

        # 영양제 모듈 트리거 (팀원 하드웨어 파트와의 인터페이스)
        self.dispense_pub = self.create_publisher(
            String, '/dispense_supplement', 10)

        # 하드웨어 모듈로부터 배출 완료 신호 수신
        self.create_subscription(
            String, '/dispense_result', self.dispense_result_callback, 10)

        # schedule_manager에게 완료 보고
        self.dose_completed_pub = self.create_publisher(
            String, '/dose_completed', 10)

        self.spin_client = ActionClient(self, Spin, 'spin')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_handle = None
        self._nav_handle = None

        self.create_timer(0.5, self.state_machine_tick)

        self.get_logger().info('영양제 전달 노드 시작됨')

    # ---------------- 미션 수신 ----------------

    def mission_callback(self, msg: String):
        if self.state != 'IDLE':
            self.get_logger().warn(
                '이미 진행 중인 미션이 있어 새 미션을 무시합니다.')
            return

        try:
            mission = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'미션 파싱 실패: {msg.data}')
            return

        self.current_mission = mission
        self.get_logger().info(
            f'미션 접수: {mission["display_name"]} '
            f'{mission["dose"]}개 전달 시작')

        self.start_searching()

    # ---------------- 상태 전환 ----------------

    def start_searching(self):
        self.state = 'SEARCHING'
        self.state_start_time = time.time()

        # yolo_world_node에게 "person"을 찾도록 지시
        target_msg = String()
        target_msg.data = 'person'
        self.search_pub.publish(target_msg)

        self.send_spin()

    def start_moving(self):
        self.state = 'MOVING'
        self.state_start_time = time.time()
        self.cancel_spin()

        cls_name, angle_deg, distance = self.latest_result
        self.send_nav_goal(angle_deg, distance)

    def start_dispensing(self):
        self.state = 'DISPENSING'
        self.state_start_time = time.time()

        dispense_msg = String()
        dispense_msg.data = json.dumps({
            'supplement_name': self.current_mission['supplement_name'],
            'display_name': self.current_mission['display_name'],
            'dose': self.current_mission['dose'],
        }, ensure_ascii=False)
        self.dispense_pub.publish(dispense_msg)

        self.get_logger().info(
            f'영양제 모듈에 배출 명령 전송: '
            f'{self.current_mission["display_name"]} '
            f'{self.current_mission["dose"]}개')

    def finish_mission(self, success):
        dose_key = self.current_mission['dose_key']

        result_msg = String()
        result_msg.data = json.dumps({
            'dose_key': dose_key,
            'success': success,
        }, ensure_ascii=False)
        self.dose_completed_pub.publish(result_msg)

        status_kr = '성공' if success else '실패'
        self.get_logger().info(f'미션 종료({status_kr}): {dose_key}')

        self.current_mission = None
        self.cancel_spin()
        self.cancel_nav()
        self.state = 'IDLE'

    # ---------------- Perception ----------------

    def perception_callback(self, msg: String):
        try:
            cls_name, angle_str, dist_str = msg.data.split('|')
            self.latest_result = (cls_name, float(angle_str), float(dist_str))
            if cls_name == 'person':
                self.last_seen_time = time.time()
        except ValueError:
            pass

    # ---------------- 상태 머신 ----------------

    def state_machine_tick(self):
        now = time.time()

        if self.state == 'SEARCHING':
            if (self.latest_result is not None
                    and self.latest_result[0] == 'person'
                    and (now - self.last_seen_time) < 1.0):
                self.get_logger().info('사용자 발견! 접근 시작')
                self.start_moving()
                return

            if (now - self.state_start_time) > self.search_timeout:
                self.get_logger().warn(
                    f'{self.search_timeout:.0f}초간 사용자를 찾지 못했습니다.')
                self.finish_mission(success=False)

        elif self.state == 'MOVING':
            if (now - self.last_seen_time) > self.lost_timeout:
                self.get_logger().warn('이동 중 사용자 시야 이탈 -> 재탐색')
                self.cancel_nav()
                self.start_searching()

        elif self.state == 'DISPENSING':
            if (now - self.state_start_time) > self.dispense_timeout:
                self.get_logger().warn(
                    '영양제 모듈 응답 시간 초과 -> 실패 처리')
                self.finish_mission(success=False)

    # ---------------- 배출 결과 수신 (팀원 하드웨어 파트) ----------------

    def dispense_result_callback(self, msg: String):
        if self.state != 'DISPENSING':
            return

        try:
            result = json.loads(msg.data)
            success = result.get('success', False)
        except json.JSONDecodeError:
            success = False

        self.finish_mission(success=success)

    # ---------------- Nav2 Spin (제자리 탐색) ----------------

    def send_spin(self):
        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('spin 액션 서버 없음')
            return
        goal = Spin.Goal()
        goal.target_yaw = 1.0
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self._spin_response)

    def _spin_response(self, future):
        handle = future.result()
        if not handle.accepted:
            return
        self._spin_handle = handle
        handle.get_result_async().add_done_callback(self._spin_result)

    def _spin_result(self, future):
        if self.state == 'SEARCHING':
            self.send_spin()

    def cancel_spin(self):
        if self._spin_handle:
            self._spin_handle.cancel_goal_async()
            self._spin_handle = None

    # ---------------- Nav2 목표 전송 ----------------

    def send_nav_goal(self, angle_deg, distance):
        target_distance = max(distance - self.approach_distance, 0.3)
        angle_rad = math.radians(angle_deg)
        x_base = target_distance * math.cos(angle_rad)
        y_base = target_distance * math.sin(angle_rad)

        point_base = PointStamped()
        point_base.header.frame_id = 'base_link'
        point_base.header.stamp = self.get_clock().now().to_msg()
        point_base.point.x = x_base
        point_base.point.y = y_base

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
        goal.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose 액션 서버 없음')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._nav_response)

        self.get_logger().info(
            f'목표 전송: map ({point_map.point.x:.2f}, '
            f'{point_map.point.y:.2f}), 원래거리={distance:.2f}m')

    def _nav_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('이동 목표가 거부되었습니다.')
            return
        self._nav_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        if self.state == 'MOVING':
            # 도착 시점에도 여전히 사람이 보이는지 확인 후 배출 단계로
            now = time.time()
            if (now - self.last_seen_time) < 2.0:
                self.get_logger().info('사용자 앞 도착 확인 -> 배출 시작')
                self.start_dispensing()
            else:
                self.get_logger().warn('도착했으나 사용자가 안 보임 -> 재탐색')
                self.start_searching()

    def cancel_nav(self):
        if self._nav_handle:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = SupplementDeliveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
