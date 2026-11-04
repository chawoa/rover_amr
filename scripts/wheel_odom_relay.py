#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

class WheelOdomRelay(Node):
    def __init__(self):
        super().__init__('wheel_odom_relay')
        self.sub = self.create_subscription(
            Twist, '/wheel_odom', self.callback, 10)
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped, '/wheel_odom_stamped', 10)

    def callback(self, msg: Twist):
        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.twist = msg

        # 공분산 설정 (선속도 신뢰, 나머지는 낮은 신뢰도)
        cov = [0.0]*36
        cov[0]  = 0.01   # vx
        cov[35] = 0.02   # vyaw
        out.twist.covariance = cov

        self.pub.publish(out)

def main():
    rclpy.init()
    node = WheelOdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
