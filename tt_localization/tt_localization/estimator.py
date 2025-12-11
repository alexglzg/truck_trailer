import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float32
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class TruckTrailerEstimator(Node):
    def __init__(self):
        super().__init__('truck_trailer_estimator')

        self.L1 = 0.537  # Hitch to Trailer Axle
        self.M0 = -0.02  # Hitch offset from Truck Axle
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/encoder/angle', self.encoder_callback, 10)

        self.state_pub = self.create_publisher(Float64MultiArray, '/truck_trailer/state', 10)
        
        self.truck_pose_pub = self.create_publisher(PoseStamped, '/truck_trailer/truck_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- INTERNAL STATE ---
        self.trailer_x = 0.0
        self.trailer_y = 0.0
        self.trailer_yaw = 0.0
        self.hitch_angle = 0.0
        self.odom_received = False

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def encoder_callback(self, msg):
        self.hitch_angle = msg.data

    def odom_callback(self, msg):
        self.trailer_x = msg.pose.pose.position.x
        self.trailer_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.trailer_yaw = self.quaternion_to_yaw(q)

        self.odom_received = True
        self.publish_full_state(msg.header.stamp)

    def publish_full_state(self, stamp):
        if not self.odom_received:
            return

        # theta0 = theta1 + beta
        truck_yaw = self.trailer_yaw + self.hitch_angle
        truck_yaw = self.normalize_angle(truck_yaw)

        hitch_x = self.trailer_x + (self.L1 * math.cos(self.trailer_yaw))
        hitch_y = self.trailer_y + (self.L1 * math.sin(self.trailer_yaw))
        
        truck_x = hitch_x + (self.M0 * math.cos(truck_yaw))
        truck_y = hitch_y + (self.M0 * math.sin(truck_yaw))

        state_msg = Float64MultiArray()
        # [TrailerX, TrailerY, TruckYaw, TrailerYaw]
        state_msg.data = [self.trailer_x, self.trailer_y, truck_yaw, self.trailer_yaw]
        self.state_pub.publish(state_msg)

        t_pose = PoseStamped()
        t_pose.header.stamp = stamp
        t_pose.header.frame_id = 'map'
        t_pose.pose.position.x = truck_x
        t_pose.pose.position.y = truck_y
        t_pose.pose.orientation.w = math.cos(truck_yaw * 0.5)
        t_pose.pose.orientation.z = math.sin(truck_yaw * 0.5)
        self.truck_pose_pub.publish(t_pose)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'truck_link'
        t.transform.translation.x = truck_x
        t.transform.translation.y = truck_y
        t.transform.translation.z = 0.0
        t.transform.rotation = t_pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TruckTrailerEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()