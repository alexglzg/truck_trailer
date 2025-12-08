#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tt_interfaces.msg import PolytopeObstacleArray, PolytopeObstacle
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        
        # Publishers
        self.mpc_pub = self.create_publisher(PolytopeObstacleArray, '/obstacles', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        
        # Define 4 Static Obstacles (x, y, width, height)
        # Note: MPC assumes these are boxes
        self.obstacles_data = [
            (1.0, 1.0, 0.6, 0.4),   # Obstacle 1 (The original prototype one)
            (3.0, 0.5, 0.5, 0.5),   # Obstacle 2
            (1.5, -1.0, 0.8, 0.3),  # Obstacle 3
            (4.0, 2.0, 0.4, 0.4)    # Obstacle 4
        ]
        
        self.timer = self.create_timer(1.0, self.publish_loop)
        self.get_logger().info("Obstacle Publisher Running.")

    def get_polytope_A_b(self, x, y, w, h):
        """
        Convert box (center x,y, width w, height h) to Ax <= b
        """
        # Box frame half-widths
        hw = w / 2.0
        hh = h / 2.0
        
        # Standard axis-aligned box normals
        # 1.  x <= x_c + hw
        # 2. -x <= - (x_c - hw)  => x >= x_c - hw
        # 3.  y <= y_c + hh
        # 4. -y <= - (y_c - hh)  => y >= y_c - hh
        
        A = np.array([
            [1.0, 0.0],
            [-1.0, 0.0],
            [0.0, 1.0],
            [0.0, -1.0]
        ])
        
        b = np.array([
            x + hw,
            -(x - hw),
            y + hh,
            -(y - hh)
        ])
        
        return A.flatten().tolist(), b.tolist()

    def publish_loop(self):
        # 1. Prepare MPC Message
        mpc_msg = PolytopeObstacleArray()
        mpc_msg.header.stamp = self.get_clock().now().to_msg()
        mpc_msg.header.frame_id = "map"
        
        # 2. Prepare Visualization Message
        viz_msg = MarkerArray()
        
        id_counter = 0
        for (x, y, w, h) in self.obstacles_data:
            # --- For MPC ---
            obs = PolytopeObstacle()
            obs.num_constraints = 4
            obs.a_vec, obs.b_vec = self.get_polytope_A_b(x, y, w, h)
            mpc_msg.obstacles.append(obs)
            
            # --- For RViz (Cube Marker) ---
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = id_counter
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.25 # Slightly elevated
            
            marker.scale.x = w
            marker.scale.y = h
            marker.scale.z = 0.5 # Height
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            viz_msg.markers.append(marker)
            id_counter += 1
            
        self.mpc_pub.publish(mpc_msg)
        self.viz_pub.publish(viz_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()