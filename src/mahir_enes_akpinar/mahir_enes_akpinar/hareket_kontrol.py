#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry

def quat_to_yaw(q):
    # Z-yönlü yaw (ENU)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def ang_normalize(a):
    # [-pi, pi]
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return a

class HareketKontrol(Node):
    def __init__(self):
        super().__init__('hareket_kontrol')

        # --------- Parametreler ---------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')                  # topic adı
        self.declare_parameter('cmd_vel_type', 'stamped')                    # 'stamped' | 'twist'
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('min_speed', 0.01)
        self.declare_parameter('stop_distance', 1.0)                         # 1 m
        self.declare_parameter('near_stop', 0.20)                            # çok yakın eşik
        self.declare_parameter('turn_speed', 0.6)                            # rad/s
        self.declare_parameter('turn_angle_deg', 30.0)                       # derece
        self.declare_parameter('front_window_deg', 30.0)                     # ön pencere (±15°)
        self.declare_parameter('sector_deg', 60.0)                           # sol/ön/sağ sektör genişliği
        self.declare_parameter('eps_ang_deg', 2.0)                           # açı epsilonu
        self.declare_parameter('log_battery', True)

        self.cmd_vel_topic   = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_vel_type    = self.get_parameter('cmd_vel_type').get_parameter_value().string_value
        self.max_speed       = float(self.get_parameter('max_speed').value)
        self.min_speed       = float(self.get_parameter('min_speed').value)
        self.stop_distance   = float(self.get_parameter('stop_distance').value)
        self.near_stop       = float(self.get_parameter('near_stop').value)
        self.turn_speed      = float(self.get_parameter('turn_speed').value)
        self.turn_angle_rad  = math.radians(float(self.get_parameter('turn_angle_deg').value))
        self.front_window    = int((float(self.get_parameter('front_window_deg').value) / 360.0) * 720)  # ~720 beam varsayımı güvenli pencere
        self.sector_width    = float(self.get_parameter('sector_deg').value)
        self.eps_ang         = math.radians(float(self.get_parameter('eps_ang_deg').value))
        self.log_battery     = bool(self.get_parameter('log_battery').value)

        # --------- QoS ---------
        sensor_qos = qos_profile_sensor_data
        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --------- Publisher: Twist veya TwistStamped ---------
        if self.cmd_vel_type.lower() == 'stamped':
            self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
            self.use_stamped = True
            self.get_logger().info(f"/cmd publisher: TwistStamped → {self.cmd_vel_topic}")
        else:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            self.use_stamped = False
            self.get_logger().info(f"/cmd publisher: Twist → {self.cmd_vel_topic}")

        # --------- Subscribers ---------
        self.scan_sub    = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.odom_sub    = self.create_subscription(Odometry, '/odom', self.odom_callback, default_qos)
        self.batt_sub    = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, sensor_qos)

        # --------- Durum ---------
        self.state = 'DRIVE'    # 'DRIVE' | 'TURN'
        self.turn_sign = 1.0     # +1 sol, -1 sağ
        self.yaw_now = 0.0
        self.yaw_target = 0.0

        self.get_logger().info("🚀 Hareket kontrol düğümü çalışıyor (Lidar+Odom+Battery).")

    # --------- Yardımcı: publish fonksiyonu ---------
    def publish_cmd(self, lin_x, ang_z):
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(lin_x)
            msg.twist.angular.z = float(ang_z)
            self.cmd_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(lin_x)
            msg.angular.z = float(ang_z)
            self.cmd_pub.publish(msg)

    # --------- Odom callback: yaw güncelle ---------
    def odom_callback(self, odom: Odometry):
        q = odom.pose.pose.orientation
        self.yaw_now = quat_to_yaw(q)

    # --------- Battery callback: isteğe bağlı log ---------
    def battery_callback(self, msg: BatteryState):
        if self.log_battery and (self.get_clock().now().nanoseconds // 1_000_000_000) % 5 == 0:
            self.get_logger().debug(f"🔋 Battery: {msg.percentage*100:.1f}%")

    # --------- Lidar callback: ana mantık ---------
    def scan_callback(self, scan: LaserScan):
        ranges = list(scan.ranges)
        rngs = [r for r in ranges if scan.range_min < r < scan.range_max]
        if not rngs:
            # sensör boş/invalid
            self.publish_cmd(0.0, 0.0)
            return

        # Ön pencere (±front_window/2 beam)
        n = len(ranges)
        center = n // 2
        half_w = max(5, min(self.front_window // 2, n // 4))
        front = [r for r in ranges[center-half_w:center+half_w] if scan.range_min < r < scan.range_max]
        min_front = min(front) if front else float('inf')

        # Sektörler (sol/ön/sağ) – beam indeksine göre yaklaşık bölme
        # Sağ: merkezden +sector_width/2..+sector_width*1.5; Sol: negatif taraf
        sector_beams = max(10, int((self.sector_width / 360.0) * n))
        right_sec = [r for r in ranges[center+5:center+5+sector_beams] if scan.range_min < r < scan.range_max]
        left_sec  = [r for r in ranges[center-(5+sector_beams):center-5] if scan.range_min < r < scan.range_max]

        avg_left  = sum(left_sec)/len(left_sec) if left_sec else float('inf')
        avg_right = sum(right_sec)/len(right_sec) if right_sec else float('inf')

        # --- Durum makinesi ---
        if self.state == 'DRIVE':
            # Engel yoksa düz ilerle
            if min_front > self.stop_distance:
                v = self.max_speed
                self.publish_cmd(v, 0.0)
                self.get_logger().info(f"🟢 İleri: v={v:.2f} m/s (min_front={min_front:.2f} m)")
                return

            # Engel var ama 0.2–1.0 m arası → orantılı yavaşla
            if self.near_stop < min_front <= self.stop_distance:
                scale = (min_front - self.near_stop) / (self.stop_distance - self.near_stop)
                v = self.min_speed + scale * (self.max_speed - self.min_speed)
                v = max(self.min_speed, min(self.max_speed, v))
                self.publish_cmd(v, 0.0)
                self.get_logger().warn(f"🟡 Yavaşla: v={v:.2f} m/s (min_front={min_front:.2f} m)")
                return

            # Engel çok yakın → dönme kararı (boş tarafa dön)
            self.turn_sign = +1.0 if avg_left > avg_right else -1.0
            self.yaw_target = ang_normalize(self.yaw_now + self.turn_sign * self.turn_angle_rad)
            self.state = 'TURN'
            self.get_logger().error(f"🔴 Engel çok yakın! DÖN (sign={'SOL' if self.turn_sign>0 else 'SAĞ'}) → hedef {math.degrees(self.yaw_target):.1f}°")

        if self.state == 'TURN':
            # Hedef açıya yaklaşana kadar dön
            ang_err = ang_normalize(self.yaw_target - self.yaw_now)
            if abs(ang_err) > self.eps_ang:
                self.publish_cmd(0.0, self.turn_sign * self.turn_speed)
                return
            # Dönüş tamam
            self.publish_cmd(0.0, 0.0)
            self.state = 'DRIVE'
            self.get_logger().info("↪️ Dönüş tamam, ileri moda geçildi.")

def main(args=None):
    rclpy.init(args=args)
    node = HareketKontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
