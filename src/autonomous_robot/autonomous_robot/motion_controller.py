import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_robot)

        self.lidar_subscriber = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, qos_profile_sensor_data)
        self.state = 'STOPPED'
        self.turn_direction = 0.0
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None
        self.last_lidar_scan_time = None

        self.odom_subscriber = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 10)
        self.x = None
        self.y = None
        self.theta = None
        self.last_odom_scan_time = None

        self.goal_x = 3.5
        self.goal_y = -1.0

        self.bypass_start_x = None
        self.bypass_start_y = None

    def lidar_callback(self, msg):
        self.last_lidar_scan_time = self.get_clock().now()
        front_min = left_min = right_min = float('inf')

        for idx, dist in enumerate(msg.ranges):
            angle = msg.angle_min + idx * msg.angle_increment
            if math.isfinite(dist) and dist >= msg.range_min and dist <= msg.range_max:
                if angle >= math.radians(-30) and angle < math.radians(30):
                    if dist < front_min:
                        front_min = dist
                elif angle >= math.radians(30) and angle <= math.radians(90):
                    if dist < left_min:
                        left_min = dist
                elif angle >= math.radians(-90) and angle < math.radians(-30):
                    if dist < right_min:
                        right_min = dist

        if math.isfinite(front_min):
            self.front_distance = front_min
        else:
            self.front_distance = None

        if math.isfinite(left_min):
            self.left_distance = left_min
        else:
            self.left_distance = None

        if math.isfinite(right_min):
            self.right_distance = right_min
        else:
            self.right_distance = None

        # self.get_logger().info(f"The minimum distance in the forward region is {front_min}")
        # self.get_logger().info(f"The minimum distance in the left region is {left_min}")
        # self.get_logger().info(f"The minimum distance in the right region is {right_min}")

    def odom_callback(self, msg):
        self.last_odom_scan_time = self.get_clock().now()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

        self.get_logger().info(f"Position: ({self.x:.2f}, {self.y:.2f}), Heading: {math.degrees(self.theta):.2f}°")

    def calculate_goal_error(self):
        if self.x is not None and self.y is not None and self.theta is not None:
            distance_error = ((self.x - self.goal_x)**2 + (self.y - self.goal_y)**2)**0.5
            desired_heading = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            heading_error = math.atan2(math.sin(desired_heading - self.theta), math.cos(desired_heading - self.theta))
            return distance_error, heading_error

    def calculate_goal_velocity(self):
        errors = self.calculate_goal_error()

        if errors is None:
            return
        distance_error, heading_error = errors

        linear_velocity = None
        angular_velocity = None

        if distance_error <= 0.15:
            linear_velocity, angular_velocity = 0.0, 0.0
        elif abs(heading_error) > 0.2:
            linear_velocity = 0.0
            angular_velocity = max(-0.6, min(0.6, 1.0 * heading_error))
        else:
            linear_velocity = min(0.3, 0.5 * distance_error)
            angular_velocity = max(-0.6, min(0.6, 1.0 * heading_error))

        return linear_velocity, angular_velocity

    def control_robot(self):
        msg = Twist()

        velocities = self.calculate_goal_velocity()

        if (
            self.last_lidar_scan_time is None or
            
            velocities is None or
            (self.get_clock().now() - self.last_lidar_scan_time > Duration(seconds=0.5))
            
        ):
            self.state = 'STOPPED'
            self.publisher_.publish(msg)
            return

        if velocities == (0.0, 0.0):
            self.state = 'GOAL_REACHED'

        if self.state == 'GOAL_REACHED':
            self.publisher_.publish(msg)
            self.get_logger().info("Goal Reached!!!")
            return

        def find_turn():
            if self.left_distance is None and self.right_distance is None:
                self.state = 'STOPPED'
                return
            elif self.left_distance is None:
                self.turn_direction = -0.5
            elif self.right_distance is None:
                self.turn_direction = 0.5
            elif self.left_distance > self.right_distance:
                self.turn_direction = 0.5
            else:
                self.turn_direction = -0.5
            self.state = 'TURNING'

        if self.front_distance is None:
            self.state = 'STOPPED'
        elif self.state == 'STOPPED':
            if self.front_distance >= 0.7:
                self.state = 'NAVIGATING'
            else:
                find_turn()
        elif self.state == 'NAVIGATING':
            if self.front_distance < 0.7:
                find_turn()
        elif self.state == 'TURNING':
            if self.front_distance > 1:
                self.state = 'BYPASSING'
                self.bypass_start_x = self.x
                self.bypass_start_y = self.y
        elif self.state == 'BYPASSING':
            if self.front_distance < 0.7:
                find_turn()
            else:
                bypass_distance = math.hypot(self.x - self.bypass_start_x, self.y - self.bypass_start_y)

                if bypass_distance >= 0.6:
                    self.state = 'NAVIGATING'

        if self.state == 'TURNING':
            msg.angular.z = self.turn_direction
        elif self.state == 'NAVIGATING':
            msg.linear.x, msg.angular.z = velocities
            self.turn_direction = 0.0
        elif self.state == 'BYPASSING':
            msg.linear.x = 0.2
            msg.angular.z = 0.0

        self.get_logger().info(
            f"State: {self.state}, "
            f"Front: {self.front_distance}, "
            f"Left: {self.left_distance}, "
            f"Right: {self.right_distance}, "
            f"Turn direction: {self.turn_direction}, "
            f"Linear: {msg.linear.x:.2f}, "
            f"Angular: {msg.angular.z:.2f}"
        )
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()