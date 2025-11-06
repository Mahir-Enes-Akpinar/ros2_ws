import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class TurtlesimSquare(Node):
    def __init__(self):
        super().__init__('turtlesim_square')

        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Parametre: kontrol modu
        self.declare_parameter('control_mode', 'open')
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value

        # Başlangıç verileri
        self.pose = Pose()
        self.state = 'go_to_start'
        self.phase_start = time.time()

        # Ortak ayarlar
        self.edge_length = 3.0
        self.tour_count = 0
        self.edge_count = 0

        # Open-loop hız ve süreler
        self.linear_speed = 1.5          # m/s
        self.angular_speed = (math.pi/2) / 1.5  # 90° için 1.5 s
        self.forward_time = self.edge_length / self.linear_speed
        self.turn_time = (math.pi/2) / self.angular_speed

        # Closed-loop hedef verileri
        self.targets = [(3,3), (6,3), (6,6), (3,6)]
        self.target_index = 0
        self.target_x, self.target_y = self.targets[self.target_index]
        self.kp_linear = 1.0
        self.kp_angular = 4.0
        self.distance_tol = 0.05
        self.angle_tol = 0.1

        # Zamanlayıcı
        self.timer = self.create_timer(0.02, self.loop)
        self.get_logger().info(f"Turtlesim kare görevi başlatıldı  |  Mod: {self.control_mode}")

    # ----------------------------------------------------------------------
    def pose_callback(self, msg):
        self.pose = msg

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ----------------------------------------------------------------------
    def loop(self):
        if self.tour_count >= 5:
            self.stop()
            self.get_logger().info('✅ 5 tur tamamlandı, görev bitti.')
            rclpy.shutdown()
            return

        if self.control_mode == 'open':
            self.open_loop_control()
        else:
            self.closed_loop_control()

    # ====================== OPEN-LOOP KONTROL ==============================
    def open_loop_control(self):
        twist = Twist()
        now = time.time()

        if not hasattr(self, 'phase_start'):
            self.phase_start = now
            self.state = 'go_to_start'

        elapsed = now - self.phase_start

        # Başlangıçta (3, 3)’e gitme — basit zaman tabanlı yaklaşım
        if self.state == 'go_to_start':
            twist.linear.x = 1.0
            if elapsed > 2.0:  # yaklaşık merkezden (5.5,5.5) → (3,3)
                self.state = 'forward'
                self.phase_start = now

        elif self.state == 'forward':
            twist.linear.x = self.linear_speed
            if elapsed >= self.forward_time:
                self.state = 'turn'
                self.phase_start = now

        elif self.state == 'turn':
            twist.angular.z = self.angular_speed
            if elapsed >= self.turn_time:
                self.edge_count += 1
                if self.edge_count % 4 == 0:
                    self.tour_count += 1
                    self.get_logger().info(f"🔁 Tur tamamlandı → {self.tour_count}")
                self.state = 'forward'
                self.phase_start = now

        self.cmd_pub.publish(twist)

    # ====================== CLOSED-LOOP KONTROL ============================
    def closed_loop_control(self):
        twist = Twist()

        # (3,3)’e varmadıysa önce oraya git
        if self.tour_count == 0 and self.target_index == 0 and self.distance_to(3,3) > 0.2:
            self.move_to_point(3.0, 3.0, twist)
            self.cmd_pub.publish(twist)
            return

        # Normal kare hareketi
        dx = self.target_x - self.pose.x
        dy = self.target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.pose.theta)

        # Açı hatası büyükse önce dön, sonra ilerle
        if abs(angle_diff) > self.angle_tol:
            twist.linear.x = 0.0
            twist.angular.z = self.kp_angular * angle_diff
        elif distance > self.distance_tol:
            twist.linear.x = self.kp_linear * distance
            twist.angular.z = self.kp_angular * angle_diff
        else:
            self.next_target()

        self.cmd_pub.publish(twist)

    # ----------------------------------------------------------------------
    def next_target(self):
        self.target_index = (self.target_index + 1) % len(self.targets)
        self.target_x, self.target_y = self.targets[self.target_index]
        self.get_logger().info(f"🎯 Yeni hedef: ({self.target_x:.1f}, {self.target_y:.1f})")

        if self.target_index == 0:
            self.tour_count += 1
            self.get_logger().info(f"🔁 Tur tamamlandı → {self.tour_count}")

    # ----------------------------------------------------------------------
    def move_to_point(self, x, y, twist):
        dx = x - self.pose.x
        dy = y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.pose.theta)

        twist.linear.x = self.kp_linear * distance
        twist.angular.z = self.kp_angular * angle_diff

    def distance_to(self, x, y):
        return math.sqrt((x - self.pose.x)**2 + (y - self.pose.y)**2)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Önce durdur, sonra node'u yok et, en sonda shutdown
        try:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
