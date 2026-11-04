import yaml
import time
import json
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ScheduleManagerNode(Node):
    def __init__(self):
        super().__init__('schedule_manager_node')

        self.declare_parameter(
            'schedule_file',
            '/home/embedded/rover_nav/config/supplement_schedule.yaml')
        self.declare_parameter(
            'log_file',
            '/home/embedded/rover_nav/config/dose_log.json')
        self.declare_parameter('check_interval_sec', 30.0)
        self.declare_parameter('grace_period_min', 60)  # 복용 시간 놓쳤을 때 유예

        schedule_file = self.get_parameter('schedule_file').value
        self.log_file = self.get_parameter('log_file').value
        self.check_interval = self.get_parameter('check_interval_sec').value
        self.grace_period_min = self.get_parameter('grace_period_min').value

        self.supplements = self.load_schedule(schedule_file)
        self.dose_log = self.load_dose_log()

        # 미션 트리거 발행 (다른 노드가 구독해서 사람 탐색/이동 시작)
        self.mission_pub = self.create_publisher(
            String, '/supplement_mission', 10)

        # 복용 완료 보고 수신 (전달 후 결과)
        self.create_subscription(
            String, '/dose_completed', self.dose_completed_callback, 10)

        self.create_timer(self.check_interval, self.check_schedule)

        self.get_logger().info(
            f'스케줄 관리자 시작됨 ({len(self.supplements)}개 영양제 등록)')

    def load_schedule(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data.get('supplements', [])

    def load_dose_log(self):
        try:
            with open(self.log_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def save_dose_log(self):
        with open(self.log_file, 'w', encoding='utf-8') as f:
            json.dump(self.dose_log, f, ensure_ascii=False, indent=2)

    def make_dose_key(self, supplement_name, scheduled_time, date_str):
        return f'{date_str}_{supplement_name}_{scheduled_time}'

    def check_schedule(self):
        now = datetime.now()
        date_str = now.strftime('%Y-%m-%d')
        current_time_str = now.strftime('%H:%M')

        for supplement in self.supplements:
            for scheduled_time in supplement['times']:
                dose_key = self.make_dose_key(
                    supplement['name'], scheduled_time, date_str)

                # 이미 오늘 이 시간에 처리(전달완료 또는 놓침처리)된 건 skip
                if dose_key in self.dose_log:
                    continue

                scheduled_dt = datetime.strptime(
                    f'{date_str} {scheduled_time}', '%Y-%m-%d %H:%M')

                elapsed_min = (now - scheduled_dt).total_seconds() / 60.0

                if elapsed_min < 0:
                    continue  # 아직 시간 안 됨

                if elapsed_min > self.grace_period_min:
                    # 유예시간 초과 - 놓친 것으로 기록
                    self.dose_log[dose_key] = {
                        'status': 'missed',
                        'supplement': supplement['name'],
                        'scheduled_time': scheduled_time,
                        'dose': supplement['dose_per_time'],
                    }
                    self.save_dose_log()
                    self.get_logger().warn(
                        f'복용 시간 놓침: {supplement["display_name"]} '
                        f'({scheduled_time})')
                    continue

                # 복용 시간 도달, 아직 미전달 -> 미션 트리거
                self.trigger_mission(supplement, scheduled_time, dose_key)

    def trigger_mission(self, supplement, scheduled_time, dose_key):
        mission_data = {
            'dose_key': dose_key,
            'supplement_name': supplement['name'],
            'display_name': supplement['display_name'],
            'dose': supplement['dose_per_time'],
            'scheduled_time': scheduled_time,
        }

        msg = String()
        msg.data = json.dumps(mission_data, ensure_ascii=False)
        self.mission_pub.publish(msg)

        # 중복 트리거 방지를 위해 "진행중" 상태로 기록
        self.dose_log[dose_key] = {
            'status': 'in_progress',
            'supplement': supplement['name'],
            'scheduled_time': scheduled_time,
            'dose': supplement['dose_per_time'],
        }
        self.save_dose_log()

        self.get_logger().info(
            f'미션 트리거: {supplement["display_name"]} '
            f'{supplement["dose_per_time"]}개 ({scheduled_time})')

    def dose_completed_callback(self, msg: String):
        try:
            result = json.loads(msg.data)
            dose_key = result.get('dose_key')
            success = result.get('success', False)

            if dose_key in self.dose_log:
                self.dose_log[dose_key]['status'] = \
                    'delivered' if success else 'failed'
                self.save_dose_log()

                status_kr = '전달 완료' if success else '전달 실패'
                self.get_logger().info(
                    f'{status_kr}: {dose_key}')
        except json.JSONDecodeError:
            self.get_logger().warn(f'파싱 실패: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
