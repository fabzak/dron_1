import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')

        self.cmd_pub = self.create_publisher(Twist, '/crazyflie/gazebo/command/twist', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.pause_sub = self.create_subscription(Bool, '/pause', self.pause_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.state = 'ROTATE'
        self.scan_data = None
        self.max_sum = 0.0
        self.best_yaw = 0.0
        self.current_yaw = 0.0
        self.rotation_start_time = self.get_clock().now()
        self.paused = False

        self.rotation_duration = 10.0
        self.move_duration = 3.0

        self.get_logger().info('ExplorerNode pokrenut.')

    def lidar_callback(self, msg):
        self.scan_data = msg

    def pause_callback(self, msg):
        self.paused = msg.data
        if self.paused:
            self.get_logger().info('PAUZA aktivirana.')
        else:
            self.get_logger().info('Nastavak izvođenja.')

    def update_current_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.current_yaw = yaw
        except:
            pass  # još nije dostupan TF

    def control_loop(self):
        if self.paused or self.scan_data is None:
            return

        self.update_current_yaw()

        twist = Twist()

        now = self.get_clock().now()
        elapsed = (now - self.rotation_start_time).nanoseconds / 1e9

        if self.state == 'ROTATE':
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)

            ranges = [r for r in self.scan_data.ranges if not math.isinf(r)]
            if ranges:
                total = sum(ranges)
                if total > self.max_sum:
                    self.max_sum = total
                    self.best_yaw = self.current_yaw

            if elapsed > self.rotation_duration:
                self.state = 'MOVE'
                self.get_logger().info(f'Skeniranje gotovo. Najotvoreniji smjer: {math.degrees(self.best_yaw):.1f}°')
                self.rotation_start_time = now

        elif self.state == 'MOVE':
            yaw_diff = self.best_yaw - self.current_yaw
            yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

            twist.linear.x = 0.4 * math.cos(yaw_diff)
            twist.linear.y = 0.4 * math.sin(yaw_diff)
            self.cmd_pub.publish(twist)

            if elapsed > self.move_duration:
                self.state = 'ROTATE'
                self.rotation_start_time = now
                self.max_sum = 0.0
        else:
            self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
