import math
import time
import json
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import Spin, NavigateToPose
from action_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class SupplementDeliveryNode(Node):
    def __init__(self):
        super().__init__('supplement_delivery_node')

        self.declare_parameter('approach_distance', 0.8)
        self.declare_parameter('local_search_timeout_sec', 15.0)
        self.declare_parameter('waypoint_search_timeout_sec', 12.0)
        self.declare_parameter('waypoint_travel_timeout_sec', 60.0)
        self.declare_parameter('lost_timeout_sec', 5.0)
        self.declare_parameter('dispense_timeout_sec', 15.0)
        self.declare_parameter(
            'waypoints_file',
            '/home/embedded/rover_nav/config/search_waypoints.yaml')

        self.approach_distance = self.get_parameter(
            'approach_distance').value
        self.local_search_timeout = self.get_parameter(
            'local_search_timeout_sec').value
        self.waypoint_search_timeout = self.get_parameter(
            'waypoint_search_timeout_sec').value
        self.waypoint_travel_timeout = self.get_parameter(
            'waypoint_travel_timeout_sec').value
        self.lost_timeout = self.get_parameter('lost_timeout_sec').value
        self.dispense_timeout = self.get_parameter(
            'dispense_timeout_sec').value

        self.waypoints = self.load_waypoints(
            self.get_parameter('waypoints_file').value)

        # 상태: IDLE, SEARCHING_LOCAL, SEARCHING_WAYPOINT,
        #       MOVING, DISPENSING
        self.state = 'IDLE'
        self.current_mission = None
        self.latest_result = None
        self.last_seen_time = 0.0
        self.state_start_time = 0.0
        self.current_waypoint_idx = -1
        self.waypoint_arrived = False
        self.latest_result_stamp = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, '/supplement_mission', self.mission_callback, 10)
        self.create_subscription(
            String, '/perception_result', self.perception_callback, 10)

        self.search_pub = self.create_publisher(String, '/search_target', 10)
        self.dispense_pub = self.create_publisher(
            String, '/dispense_supplement', 10)
        self.create_subscription(
            String, '/dispense_result', self.dispense_result_callback, 10)
        self.dose_completed_pub = self.create_publisher(
            String, '/dose_completed', 10)

        self.spin_client = ActionClient(self, Spin, 'spin')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_handle = None
        self._nav_handle = None

        self.create_timer(0.5, self.state_machine_tick)

        self.get_logger().info(
            f'영양제 전달 노드 시작됨 (웨이포인트 {len(self.waypoints)}개 로드)')

    # ---------------- 웨이포인트 로드 ----------------

    def load_waypoints(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            return data.get('waypoints', [])
        except FileNotFoundError:
            self.get_logger().warn(
                f'웨이포인트 파일 없음: {path}, 빈 목록으로 시작')
            return []

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

        self.start_local_search()

    # ---------------- 상태 전환: 1단계 제자리 탐색 ----------------

    def start_local_search(self):
        self.state = 'SEARCHING_LOCAL'
        self.state_start_time = time.time()
        self.search_pub.publish(String(data='person'))
        self.send_spin()
        self.get_logger().info('제자리 탐색 시작')

    # ---------------- 상태 전환: 2단계 웨이포인트 순회 ----------------

    def start_waypoint_search(self):
        if not self.waypoints:
            self.get_logger().warn('웨이포인트가 없어 탐색 실패 처리')
            self.finish_mission(success=False)
            return

        self.state = 'SEARCHING_WAYPOINT'
        self.current_waypoint_idx = 0
        self.move_to_current_waypoint()

    def move_to_current_waypoint(self):
        wp = self.waypoints[self.current_waypoint_idx]
        self.waypoint_arrived = False
        self.state_start_time = time.time()
        self.send_nav_goal_absolute(wp['x'], wp['y'], wp.get('yaw', 0.0))
        self.get_logger().info(
            f"웨이포인트 이동: {wp['name']} "
            f"({self.current_waypoint_idx + 1}/{len(self.waypoints)})")

    def advance_to_next_waypoint(self):
        self.current_waypoint_idx += 1
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().warn(
                '모든 웨이포인트를 순회했으나 사용자를 찾지 못함')
            self.finish_mission(success=False)
            return
        self.move_to_current_waypoint()

    # ---------------- 상태 전환: 이동/배출 ----------------

    def start_moving(self):
        self.state = 'MOVING'
        self.state_start_time = time.time()
        self.approaching_close = False
        self.cancel_spin()
        cls_name, angle_deg, distance = self.latest_result
        self.approach_start_track_id = self.latest_track_id
        self.send_nav_goal_relative(angle_deg, distance)
        self.get_logger().info('사용자 발견! 접근 시작')

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
        self.current_waypoint_idx = -1
        self.waypoint_arrived = False
        self.cancel_spin()
        self.cancel_nav()
        self.state = 'IDLE'

    # ---------------- Perception ----------------

    def perception_callback(self, msg: String):
        try:
            parts = msg.data.split('|')
            cls_name = parts[0]
            angle = float(parts[1])
            dist = float(parts[2])
            stamp_sec = int(parts[3])
            stamp_nanosec = int(parts[4])
            track_id = int(parts[5]) if len(parts) > 5 else -1

            self.latest_result = (cls_name, angle, dist)
            self.latest_result_stamp = rclpy.time.Time(
                seconds=stamp_sec, nanoseconds=stamp_nanosec)
            self.latest_track_id = track_id

            if cls_name == 'person':
                self.last_seen_time = time.time()
                if self.state in ('SEARCHING_LOCAL', 'SEARCHING_WAYPOINT'):
                    self.cancel_spin()
        except (ValueError, IndexError):
            pass

    def _person_visible(self, now):
        return (self.latest_result is not None
                and self.latest_result[0] == 'person'
                and (now - self.last_seen_time) < 1.0)

    # ---------------- 상태 머신 ----------------

    def state_machine_tick(self):
        now = time.time()

        if self.state == 'SEARCHING_LOCAL':
            if self._person_visible(now):
                self.start_moving()
                return
            if (now - self.state_start_time) > self.local_search_timeout:
                self.get_logger().info(
                    '제자리 탐색 실패 -> 웨이포인트 순회 시작')
                self.start_waypoint_search()

        elif self.state == 'SEARCHING_WAYPOINT':
            if self._person_visible(now):
                self.start_moving()
                return

            # 아직 도착 안 했으면(이동 중이면) 재탐색 타임아웃을 체크하지 않음
            if not self.waypoint_arrived:
                # 이동 자체가 비정상적으로 오래 걸리는 경우를 대비한 안전장치
                if (now - self.state_start_time) > self.waypoint_travel_timeout:
                    self.get_logger().warn(
                        '웨이포인트 이동 시간 초과 -> 다음 웨이포인트로')
                    self.cancel_nav()
                    self.advance_to_next_waypoint()
                return

            # 도착한 뒤, 그 자리에서 재탐색 시간이 지나면 다음 웨이포인트로
            if (now - self.state_start_time) > self.waypoint_search_timeout:
                self.advance_to_next_waypoint()

        elif self.state == 'MOVING':
            pass

        elif self.state == 'DISPENSING':
            if (now - self.state_start_time) > self.dispense_timeout:
                self.get_logger().warn(
                    '영양제 모듈 응답 시간 초과 -> 실패 처리')
                self.finish_mission(success=False)

    # ---------------- 배출 결과 수신 ----------------

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
        goal.target_yaw = 6.28  # 한 바퀴
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self._spin_response)

    def _spin_response(self, future):
        handle = future.result()
        if not handle.accepted:
            return
        self._spin_handle = handle
        handle.get_result_async().add_done_callback(self._spin_result)

    def _spin_result(self, future):
        if self.state in ('SEARCHING_LOCAL', 'SEARCHING_WAYPOINT'):
            self.send_spin()

    def cancel_spin(self):
        if self._spin_handle:
            self._spin_handle.cancel_goal_async()
            self._spin_handle = None

    # ---------------- Nav2 목표 전송 (절대 좌표: 웨이포인트용) ----------------

    def send_nav_goal_absolute(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose 액션 서버 없음')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._waypoint_nav_response)

    def _waypoint_nav_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('웨이포인트 목표가 거부되었습니다.')
            self.advance_to_next_waypoint()
            return
        self._nav_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._waypoint_nav_result)

    def _waypoint_nav_result(self, future):
        if self.state != 'SEARCHING_WAYPOINT':
            return

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.waypoint_arrived = True
            self.state_start_time = time.time()
            self.search_pub.publish(String(data='person'))
            self.send_spin()
            self.get_logger().info('웨이포인트 도착 성공, 재탐색 시작')
        else:
            self.get_logger().warn(
                f'웨이포인트 이동 실패(status={status}) -> 다음 웨이포인트로')
            self.advance_to_next_waypoint()

    # ---------------- Nav2 목표 전송 (상대 좌표: 사용자 접근용) ----------------

    def send_nav_goal_relative(self, angle_deg, distance):
        target_distance = max(distance - self.approach_distance, 0.3)
        angle_rad = math.radians(angle_deg)
        x_base = target_distance * math.cos(angle_rad)
        y_base = -target_distance * math.sin(angle_rad)

        point_base = PointStamped()
        point_base.header.frame_id = 'base_link'
        point_base.header.stamp = self.get_clock().now().to_msg()
        point_base.point.x = x_base
        point_base.point.y = y_base

        lookup_time = (self.latest_result_stamp
                       if self.latest_result_stamp is not None
                       else rclpy.time.Time())

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', lookup_time,
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
        future.add_done_callback(self._approach_nav_response)

        self.get_logger().info(
            f'목표 전송: map ({point_map.point.x:.2f}, '
            f'{point_map.point.y:.2f})')

    def _approach_nav_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('접근 목표가 거부되었습니다.')
            return
        self._nav_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._approach_nav_result)

    def _approach_nav_result(self, future):
        if self.state != 'MOVING':
            return

        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 지점 도착 -> 배출 시작')
            self.start_dispensing()
        else:
            self.get_logger().warn(f'접근 실패(status={status}) -> 재탐색')
            self.start_local_search()


    def _verify_arrival_near_user(self, now):
        # 조건 1: 최근(3초 이내)에 person이 확인됐어야 함
        recently_seen = (now - self.last_seen_time) < 3.0

        # 조건 2: 마지막 확인된 거리가 충분히 가까웠어야 함
        close_enough = False
        if self.latest_result is not None:
            cls_name, angle_deg, distance = self.latest_result
            if cls_name == 'person':
                close_enough = distance <= (self.approach_distance + 0.5)

        # 조건 3: 같은 트랙 ID를 이 접근 과정 내내 
        # 유지하고 있었는지 (선택적, 신뢰도 강화용)
        same_track = (self.approach_start_track_id is None or
                      self.latest_track_id == self.approach_start_track_id)

        return recently_seen and close_enough and same_track

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
