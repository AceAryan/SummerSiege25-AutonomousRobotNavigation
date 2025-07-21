import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class APFNavigator(Node):
    def __init__(self):
        super().__init__('apf_navigator')
        self.goal = np.array([3.0, 3.0])  # Set your goal here
        self.position = np.zeros(2)
        self.yaw = 0.0
        self.laser_ranges = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        # Yaw extraction from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.laser_ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def control_loop(self):
        if self.laser_ranges is None:
            return
        # Attractive force
        to_goal = self.goal - self.position
        dist_to_goal = np.linalg.norm(to_goal)
        if dist_to_goal < 0.3:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Goal reached!')
            return
        att_force = 1.0 * to_goal / (dist_to_goal + 1e-6)
        # Repulsive force
        rep_force = np.zeros(2)
        for i, r in enumerate(self.laser_ranges):
            if np.isinf(r) or np.isnan(r) or r > 1.0:
                continue
            angle = self.angle_min + i * self.angle_increment + self.yaw
            obs_vec = np.array([np.cos(angle), np.sin(angle)])
            rep_force -= 0.5 * (1.0/r - 1.0/1.0) * (1.0/(r**2)) * obs_vec
        # Total force
        total_force = att_force + rep_force
        # Convert to robot frame
        angle_to_goal = np.arctan2(total_force[1], total_force[0])
        angle_diff = angle_to_goal - self.yaw
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        cmd = Twist()
        cmd.linear.x = min(0.3, np.linalg.norm(total_force))
        cmd.angular.z = 1.0 * angle_diff
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = APFNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
