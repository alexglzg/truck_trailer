#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class StraightLineHumanSimulator(Node):
    def __init__(self):
        super().__init__('straight_line_human_simulator')
        self.pub = self.create_publisher(Odometry, '/human1/odom_smoothed', 10)
        
        # Publishing rate
        self.rate_hz = 10.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_callback)
        
        self.start_time = self.get_clock().now()
        
        # --- Simulation Parameters ---
        self.start_x = 0.0          # Starting X coordinate [m]
        self.start_y = 2.5          # Starting Y coordinate [m]
        self.heading = 0.0 #-1.5708       # Heading angle [rad] (0.0 = straight along X-axis)
        self.speed = 0.3            # Constant forward body speed [m/s]

        self.get_logger().info(
            f"Straight-line simulator started. Speed: {self.speed}m/s, "
            f"Heading: {self.heading}rad. Publishing to /human1/odom_smoothed"
        )

    def _timer_callback(self):
        # Calculate elapsed time in seconds
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9

        # 1. Calculate Global Position based on constant velocity and heading
        x = self.start_x + (self.speed * math.cos(self.heading) * t)
        y = self.start_y + (self.speed * math.sin(self.heading) * t)

        # 2. Convert constant heading angle to Quaternion
        qz = math.sin(self.heading / 2.0)
        qw = math.cos(self.heading / 2.0)

        # 3. Construct the Odometry Message
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'human1'

        # Set Pose (Global frame)
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        # Set Twist (Expressed in the child/body frame)
        msg.twist.twist.linear.x = float(self.speed)
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = 0.0

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StraightLineHumanSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()