import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class HeartDrawer(Node):
    def __init__(self):
        super().__init__('heart_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        time.sleep(1.0)
        self.draw_heart()

    def draw_heart(self):
        self.get_logger().info("❤️ Kalp çizimi başlıyor...")

        twist = Twist()
        rate = 0.05  # frame süresi (saniye)
        speed = 2.0  # linear hız katsayısı
        t = 0.0

        while t < 2 * math.pi:
            # matematiksel kalp eğrisi
            x = 16 * math.sin(t)**3
            y = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)

            # türevleri: yön ve hız için
            dx = 48 * (math.sin(t)**2) * math.cos(t)
            dy = -13 * math.sin(t) + 10 * math.sin(2*t) + 6 * math.sin(3*t) + 4 * math.sin(4*t)

            angle = math.atan2(dy, dx)
            twist.linear.x = speed * 0.3      # sabit hız
            twist.angular.z = math.sin(t * 2) * 1.2  # dönüşler için kıvrım efekti

            self.publisher_.publish(twist)
            time.sleep(rate)
            t += 0.05

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("✅ Kalp çizimi tamamlandı!")

def main(args=None):
    rclpy.init(args=args)
    node = HeartDrawer()
    node.destroy_node()
    rclpy.shutdown()
