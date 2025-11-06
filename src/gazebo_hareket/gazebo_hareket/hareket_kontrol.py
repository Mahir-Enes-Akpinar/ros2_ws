import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random

class HareketKontrol(Node):
    def __init__(self):
        super().__init__('hareket_kontrol')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.min_dist = 999.0
        self.yaw = 0.0
        self.target_yaw = None
        self.rotating = False
        self.eps = 0.02

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🚀 hareket_kontrol düğümü başlatıldı!")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        # ön taraf yaklaşık 20 derece aralığında
        ranges = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        valid_ranges = [r for r in ranges if not math.isinf(r)]
        if valid_ranges:
            self.min_dist = min(valid_ranges)

    def control_loop(self):
        twist = Twist()

        if self.min_dist > 1.0 and not self.rotating:
            twist.linear.x = 0.22
            twist.angular.z = 0.0
        else:
            twist.linear.x = max(0.01, 0.22 * (self.min_dist / 1.0))
            if not self.rotating:
                self.rotating = True
                direction = random.choice([-1, 1])
                self.target_yaw = self.yaw + direction * (math.pi / 2)
                self.get_logger().info(f"🧱 Engel algılandı ({self.min_dist:.2f} m), {'sağa' if direction==-1 else 'sola'} dönülüyor...")

        if self.rotating:
            error = abs(self.target_yaw - self.yaw)
            if error > self.eps:
                twist.linear.x = 0.0
                twist.angular.z = 0.3
            else:
                self.rotating = False
                twist.angular.z = 0.0
                self.get_logger().info("✅ Yeni yöne dönüldü.")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HareketKontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
