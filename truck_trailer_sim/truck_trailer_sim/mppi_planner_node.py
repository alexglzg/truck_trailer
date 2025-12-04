#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Quaternion, PoseWithCovarianceStamped

import yaml
import math
import torch
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(__file__))  # adds current directory

pkg_share = get_package_share_directory('truck_trailer_sim')
yaml_path = os.path.join(pkg_share, 'cfg', 'cfg_truck_trailer.yaml')

from mppi_torch.mppi_torch.mppi import MPPIPlanner
from dynamics import Dynamics
from objective import Objective

class truckTrailerPlanner(Node):
    def __init__(self):
        super().__init__('truck_trailer_mppi_planner')
        self.get_logger().info('truck-Trailer Planner Started')

        # subscribe to 2D Nav Goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10
        )

        # subscribe to the /map topic 
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.map_data = None  # holds the latest converted OccupancyGrid

        # subscribe to the /state topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/truck_trailer/state',
            self.state_callback,
            10
        )
        # State: [x1, y1, theta0, theta1]
        # x1, y1: trailer position
        # theta0: truck orientation
        # theta1: trailer orientation
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)

        # Planner will be initialized once map is received
        self.planner_initialized = False
        self.planner = None
        self.map_received = False
        self.goal = None

        # Simulation timer (20 Hz)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.run_planner)
    

    def goal_callback(self, msg: PoseStamped):
        # extract goal x,y from RViz click
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.goal = [x, y]
        self.get_logger().info(f"Received new goal from RViz: {self.goal}")

        # If the map is already received and planner not initialized, initialize
        if self.map_received and not self.planner_initialized:
            self.initialize_planner()
        
    
    def map_callback(self, msg):
        self.get_logger().info(
            f"Received occupancy grid: {msg.info.width} x {msg.info.height}"
        )
        self.map_data = self.preprocess_occupancy_grid(msg, device='cuda')

        if self.goal is not None and not self.planner_initialized:
            self.initialize_planner()

        # if not self.planner_initialized:
        #     self.initialize_planner()
        #     self.planner_initialized = True
        #     self.get_logger().info('Planner initialized after receiving map.')

        
    def state_callback(self, msg):
        self.state[0] = msg.data[0]
        self.state[1] = msg.data[1]
        self.state[2] = msg.data[2]
        self.state[3] = msg.data[3]
        
        # self.get_logger().warn(
        #     f'received pose is x: {self.state[0]:.2f}, y: {self.state[1]:.2f}, theta: {self.state[3]:.2f} rad'
        # )
    
    def publish_action(self, action):        
        # Convert to float (in case it's a tensor)
        lin_vel = float(action[0])
        ang_vel = float(action[1])

        # Create Twist message
        msg = Twist()
        msg.linear.x = lin_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = ang_vel

        # Publish
        self.cmd_pub.publish(msg)
    
    def run_planner(self):
        if self.planner is None:
            return  # wait until planner is initialized
        
        action = self.planner.command(self.state)
        self.publish_action(action)

        # self.get_logger().info(f'Planner output: {action}')


    def load_mppi_config(self):
        with open(yaml_path, "r") as file:
            config = yaml.safe_load(file)
        return config
    
    def initialize_planner(self):
        
        self.get_logger().info("Initializing MPPI planner...")
        # Load configuration
        self.cfg = self.load_mppi_config()

        # Initialize dynamics
        self.dynamics = Dynamics(dt=self.cfg["dt"], device=self.cfg["device"])

        # Define goal
        # self.goal = [2, 2]

        # Define objective with the received map
        objective = Objective(
            goal=self.goal,
            pre_processed_map=self.map_data,
            device=self.cfg["device"]
        )

        # Initialize MPPI planner
        self.planner = MPPIPlanner(
            cfg=self.cfg["mppi"],
            nx=4,
            dynamics=self.dynamics.step,
            running_cost=objective.compute_running_cost,
        )

        self.planner_initialized = True
        self.get_logger().info("MPPI planner initialized successfully.")

    
    def preprocess_occupancy_grid(self, grid_msg, device='cuda:0'):
        """
        Convert OccupancyGrid -> PyTorch tensor + metadata.
        """
        width  = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution

        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        # get map yaw
        q = grid_msg.info.origin.orientation
        yaw = self.quaternion_to_yaw((q.x, q.y, q.z, q.w))

        # flatten data -> reshape -> convert to torch
        occ_np = np.array(grid_msg.data, dtype=np.int8).reshape(height, width)
        
        # occupancy mask: 1 = obstacle, 0 = free
        occ_mask = torch.from_numpy((occ_np > 50).astype(np.uint8)).to(device)

        # print("map origin is: ", torch.tensor([origin_x, origin_y], device=device))
        # print("resolution is: ", resolution)
        # print("width is: ", width)
        # print("height is: ", height)
        # print("occ_mask is: ", occ_mask.shape)
        # print("yaw is: ", yaw)

        return {
            'occ': occ_mask,                 # (H, W)
            'origin': torch.tensor([origin_x, origin_y], device=device),
            'resolution': torch.tensor([resolution], device=device),
            'yaw': torch.tensor([yaw], device=device),
            'W': width,
            'H': height,
        }
    

    def quaternion_to_yaw(self, q):
        """
        Convert quaternion (x,y,z,w) to yaw in radians.
        """
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)
    

def main(args=None):
    rclpy.init(args=args)           # start ros2 communication
    node = truckTrailerPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()