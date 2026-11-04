import json
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama


class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')

        self.declare_parameter('model_name', 'qwen2.5:3b')
        self.model_name = self.get_parameter('model_name').value

        self.command_sub = self.create_subscription(
            String, '/natural_command', self.command_callback, 10)

        self.search_pub = self.create_publisher(
            String, '/search_target', 10)
        self.goto_pub = self.create_publisher(
            String, '/go_to_target', 10)

        self.get_logger().info(f'명령 파싱 노드 시작됨 (모델: {self.model_name})')
        self.get_logger().info(
            '예시: ros2 topic pub /natural_command std_msgs/msg/String '
            '"data: \'저기 있는 핸드폰 좀 가져다줘\'" --once')

    def build_prompt(self, user_text):
        return (
            "다음 문장에서 로봇이 찾아가거나 다뤄야 할 대상을 "
            "영어 단어로 추출해. 사람이면 'person', 물건이면 "
            "해당 영어 단어를 써. 반드시 JSON만 출력해.\n\n"
            "예시:\n"
            "문장: '저기 상자 좀 가져다줘' → {\"target\": \"box\"}\n"
            "문장: '저 사람한테 가줘' → {\"target\": \"person\"}\n"
            "문장: '물병 찾아줘' → {\"target\": \"bottle\"}\n\n"
            f'문장: "{user_text}"'
        )

    def extract_target(self, user_text):
        prompt = self.build_prompt(user_text)

        response = ollama.chat(
            model=self.model_name,
            messages=[{'role': 'user', 'content': prompt}]
        )

        reply_text = response['message']['content'].strip()

        match = re.search(r'\{[^}]*\}', reply_text)
        if not match:
            raise ValueError(f'JSON을 찾을 수 없음: {reply_text}')

        parsed = json.loads(match.group(0))
        return parsed.get('target', '').strip()

    def command_callback(self, msg: String):
        user_text = msg.data.strip()
        self.get_logger().info(f'명령 수신: "{user_text}"')

        try:
            target = self.extract_target(user_text)

            if not target:
                self.get_logger().warn('물체를 추출하지 못했습니다.')
                return

            self.get_logger().info(f'추출된 목표: "{target}"')

            search_msg = String()
            search_msg.data = target
            self.search_pub.publish(search_msg)

            # 일회성 타이머로 변경
            timer = self.create_timer(2.0, lambda: self.send_goto_once(target))
            self._pending_timer = timer  # 참조 유지 (GC 방지)

        except Exception as e:
            self.get_logger().error(f'명령 파싱 실패: {e}')

    def send_goto_once(self, target):
        goto_msg = String()
        goto_msg.data = target
        self.goto_pub.publish(goto_msg)
        self.get_logger().info(f'"{target}"로 이동 명령 전송')
        self._pending_timer.cancel()  # 실행 후 즉시 타이머 취소


def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
