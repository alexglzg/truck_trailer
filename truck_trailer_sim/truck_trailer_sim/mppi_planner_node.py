#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Path

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
        self.path_pub  = self.create_publisher(Path, '/planned_path', 10)

        # Planner will be initialized once map is received
        self.planner_initialized = False
        self.planner = None
        self.map_received = False
        self.goal = None

        # Simulation timer (10 Hz)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.run_planner)
    

    def goal_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.goal = [x, y]
        self.get_logger().info(f"Received new goal from RViz: {self.goal}")

        if self.map_received and not self.planner_initialized:
            self.initialize_planner()
            
        elif self.planner_initialized:
            self.get_logger().info("Updating planner goal...")
            
            # 1. Update the goal tensor in the Objective
            # Note: We match the device ('cpu') and format used in Objective.__init__
            device = self.cfg['device']
            new_goal_tensor = torch.tensor(self.goal, device=device, dtype=torch.float32).unsqueeze(0)
            self.objective.nav_goal = new_goal_tensor
            
            # 2. Reset the MPPI "Warm Start"
            # MPPI remembers the previous trajectory (mean_action) to speed up convergence.
            # When the goal changes dramatically, we must wipe this memory, 
            # otherwise it tries to bend the old path instead of finding a new one.
            if hasattr(self.planner, 'mean_action'):
                self.planner.mean_action.zero_()
        
    
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
        steer_angle = float(action[1])

        # ang_vel = lin_vel * math.tan(steer_angle) / Dynamics()._L0  # omega = v * tan(delta) / L

        # Create Twist message
        msg = Twist()
        msg.linear.x = lin_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = steer_angle

        # Publish
        self.cmd_pub.publish(msg)

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(path_points.shape[0]):
            point = path_points[i]
            pose = PoseStamped()
            pose.header = path_msg.header
            
            # --- CHANGE THESE LINES ---
            # Cast numpy types to python float to satisfy ROS 2 Foxy
            pose.pose.position.x = float(point[0]) 
            pose.pose.position.y = float(point[1])
            # --------------------------
            
            pose.pose.position.z = 0.0
            
            # Use index 3 for theta (orientation)
            theta = float(point[3])
            quat = Quaternion()
            quat.z = math.sin(theta / 2.0)
            quat.w = math.cos(theta / 2.0)
            pose.pose.orientation = quat
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
    
    def run_planner(self):
        if self.planner is None:
            return  # wait until planner is initialized
        
        action = self.planner.command(self.state)
        self.publish_action(action)

        best_states, _ = self.planner.get_n_best_samples(1)
        path_np = best_states.squeeze(0).cpu().numpy()
        self.publish_path(path_np)

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
        self.objective = Objective(
            goal=self.goal,
            pre_processed_map=self.map_data,
            device=self.cfg["device"]
        )

        # Initialize MPPI planner
        self.planner = MPPIPlanner(
            cfg=self.cfg["mppi"],
            nx=4,
            dynamics=self.dynamics.step,
            running_cost=self.objective.compute_running_cost,
        )

        self.planner_initialized = True
        self.get_logger().info("MPPI planner initialized successfully.")

    def preprocess_occupancy_grid(self, grid_msg, device='cuda'):
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