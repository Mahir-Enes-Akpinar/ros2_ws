import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class HeartDrawer(Node):
    def __init__(self):
        super().__init__('heart_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.rate = self.create_rate(50)  # 50 Hz
        time.sleep(1.0)

        self.draw_heart()

    def draw_heart(self):
        self.get_logger().info('Kalp çizimi başlıyor ❤️')

        twist = Twist()
        scale = 0.2  # küçültme faktörü (turtlesim ekranına sığması için)
        t_prev = 0.0
        x_prev = 16 * math.sin(t_prev) ** 3
        y_prev = 13 * math.cos(t_prev) - 5 * math.cos(2 * t_prev) - 2 * math.cos(3 * t_prev) - math.cos(4 * t_prev)

        for t in [i * 0.1 for i in range(0, 630)]:  # 0'dan 63.0'a kadar (~2π * 10)
            x = 16 * math.sin(t) ** 3
            y = 13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)

            dx = x - x_prev
            dy = y - y_prev

            distance = math.sqrt(dx ** 2 + dy ** 2)
            angle = math.atan2(dy, dx)

            twist.linear.x = distance * scale * 2.5
            twist.angular.z = angle * scale * 0.5

            self.publisher_.publish(twist)
            time.sleep(0.05)

            x_prev = x
            y_prev = y

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Kalp çizimi tamamlandı ❤️')


def main(args=None):
    rclpy.init(args=args)
    node = HeartDrawer()
    node.destroy_node()
    rclpy.shutdown()
