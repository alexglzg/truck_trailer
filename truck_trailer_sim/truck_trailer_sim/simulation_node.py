#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
from numpy import sin, cos, tan
import math
from ament_index_python.packages import get_package_share_directory
# import yaml
# import torch

import sys
import os
sys.path.append(os.path.dirname(__file__))  # adds current directory

# from mppi_torch.mppi_torch.mppi import MPPIPlanner
# from dynamics import Dynamics
# from objective import Objective


class truckTrailerSimulator(Node):
    def __init__(self):
        super().__init__('truck_trailer_simulator')
        self.get_logger().info('truck-Trailer Simulator Started')
        
        # System parameters (matching your real system)
        self.L0 = 0.42   # truck wheelbase
        self.M0 = -0.02   # hitch offset behind truck rear axle
        self.L1 = 0.537   # hitch to trailer axle

        
        # State: [x1, y1, theta0, theta1, phi]
        # x1, y1: trailer position
        # theta0: truck orientation
        # theta1: trailer orientation
        # phi: front wheel steering angle
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Control inputs
        self.vk = 0.0      # kelo velocity
        self.wk = 0.0      # kelo angular velocity
        
        # Publishers 
        self.state_pub = self.create_publisher(Float64MultiArray, '/truck_trailer/state', 10)
        self.trailer_pose_pub = self.create_publisher(PoseStamped, '/truck_trailer/trailer_pose', 10)
        self.truck_pose_pub = self.create_publisher(PoseStamped, '/truck_trailer/truck_pose', 10)
        self.encoder_pub = self.create_publisher(Float32, '/encoder/angle', 10) # Hitch angle
        self.kelo_angle_pub = self.create_publisher(Float32, '/kelo/angle', 10) # Kelo angle
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        #Subscribers
        # NOTE: We treat linear.x as Kelo Speed, angular.z as Kelo Rotation Speed
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        
        # Simulation timer (20 Hz)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.simulation_step)
        self.last_time = self.get_clock().now()
    
        
    def cmd_callback(self, msg):
        """
        Receive control commands for the Kelo Actuator.
        msg.linear.x  -> Kelo Linear Velocity (vk)
        msg.angular.z -> Kelo Angular Velocity (wk)
        """
        self.vk = max(0.0, msg.linear.x) # Kelo cannot go backwards
        self.wk = msg.angular.z

    def initialpose_callback(self, msg):
        """Set initial pose from Rviz2 2D Pose Estimate"""
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.state[2] = yaw      # theta0 (truck orientation)
        self.state[3] = yaw      # theta1 (trailer orientation)
        self.state[4] = 0.0      # phi (front wheel angle)
        
        self.get_logger().warn(
            f'Initial pose set to x: {self.state[0]:.2f}, y: {self.state[1]:.2f}, theta: {yaw:.2f} rad'
        )
    
    def truck_trailer_dynamics(self, state, u):
        """
        Compute state derivatives using kinematic model.

        state = [x1, y1, theta0, theta1, phi]
        u = [vk, wk]
        """
        x1, y1, th0, th1, phi = state
        vk, wk = u
        
        beta = th0 - th1 # Hitch angle
        V0 = vk * cos(phi)  # truck velocity
        w0 = vk * sin(phi) / self.L0  # truck angular velocity
        
        # Trailer velocity (kinematic relation)
        V1 = V0 * cos(beta) + self.M0 * w0 * sin(beta)
        
        # State derivatives
        dx1 = V1 * cos(th1)
        dy1 = V1 * sin(th1)
        dth0 = w0
        dth1 = (V0 * sin(beta)) / self.L1 - \
               (self.M0 * w0 * cos(beta)) / self.L1
        dphi = wk

        return np.array([dx1, dy1, dth0, dth1, dphi])
    
    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        q = Quaternion()
        q.w = cy
        q.x = 0.0
        q.y = 0.0
        q.z = sy
        return q
    
    def simulation_step(self):
        """Main simulation loop"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Integrate dynamics (Euler integration)
        u = np.array([self.vk, self.wk])
        state_dot = self.truck_trailer_dynamics(self.state, u)
        self.state = self.state + self.dt * state_dot
        
        # Unwrap angles to [-pi, pi]
        self.state[2] = self.normalize_angle(self.state[2])  # theta0
        self.state[3] = self.normalize_angle(self.state[3])  # theta1
        self.state[4] = self.normalize_angle(self.state[4])  # phi
        
        # Publish all topics
        self.publish_state(now)
        self.publish_transforms(now)
        
        self.last_time = now

    
    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


    def publish_state(self, now):
        """Publish state in same format as hardware interface"""
        x1, y1, th0, th1, phi = self.state
        
        # State vector [x1, y1, theta0, theta1, phi]
        state_msg = Float64MultiArray()
        state_msg.data = [float(x1), float(y1), float(th0), float(th1), float(phi)]
        self.state_pub.publish(state_msg)
        
        # Encoder angle (hitch angle beta = theta0 - theta1)
        encoder_msg = Float32()
        encoder_msg.data = float(th0 - th1)
        self.encoder_pub.publish(encoder_msg)
        
        # Trailer pose (matching hardware interface)
        trailer_msg = PoseStamped()
        trailer_msg.header.stamp = now.to_msg()
        trailer_msg.header.frame_id = 'map'
        trailer_msg.pose.position.x = float(x1)
        trailer_msg.pose.position.y = float(y1)
        trailer_msg.pose.position.z = 0.0
        trailer_msg.pose.orientation = self.euler_to_quaternion(th1)
        self.trailer_pose_pub.publish(trailer_msg)
        
        # truck pose (computed from geometry, matching hardware)
        x0 = x1 + self.L1*cos(th1) + self.M0*cos(th0)
        y0 = y1 + self.L1*sin(th1) + self.M0*sin(th0)
        
        truck_msg = PoseStamped()
        truck_msg.header.stamp = now.to_msg()
        truck_msg.header.frame_id = 'map'
        truck_msg.pose.position.x = float(x0)
        truck_msg.pose.position.y = float(y0)
        truck_msg.pose.position.z = 0.0
        truck_msg.pose.orientation = self.euler_to_quaternion(th0)
        self.truck_pose_pub.publish(truck_msg)
        
        # Odometry (for RViz)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'trailer_link'
        odom.pose.pose.position.x = float(x1)
        odom.pose.pose.position.y = float(y1)
        odom.pose.pose.orientation = self.euler_to_quaternion(th1)
        odom.twist.twist.linear.x = float(self.vk*cos(phi))
        odom.twist.twist.angular.z = float(phi)
        self.odom_pub.publish(odom)

        """Publish joint states for hitch articulation and front wheel steering"""
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['hitch_joint', 'front_wheel_joint']
        js.position = [
            float(th0 - th1),    # Hitch articulation angle (beta)
            float(phi)           # Front wheel steering angle
        ]
        self.joint_pub.publish(js)


    def publish_transforms(self, now):
        """Publish all TF transforms"""
        x1, y1, th0, th1, phi = self.state
                
        transforms = []
                
        # Map -> Trailer
        t_trailer = TransformStamped()
        t_trailer.header.stamp = now.to_msg()
        t_trailer.header.frame_id = 'map'
        t_trailer.child_frame_id = 'trailer_link'
        t_trailer.transform.translation.x = float(x1)
        t_trailer.transform.translation.y = float(y1)
        t_trailer.transform.translation.z = 0.0
        t_trailer.transform.rotation = self.euler_to_quaternion(th1)
        transforms.append(t_trailer)
        
        
        # Send all transforms at once
        self.tf_broadcaster.sendTransform(transforms)
    
    def quaternion_to_yaw(self, q):
        """
        Convert quaternion (x,y,z,w) to yaw in radians.
        """
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = truckTrailerSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()