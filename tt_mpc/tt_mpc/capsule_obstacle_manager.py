#!/usr/bin/env python3
"""
capsule_obstacle_manager.py  --  perception -> CapsuleObstacleArray.

For each tracked source (a nav_msgs/Odometry, e.g. /human1/odom_smoothed from
robot_localization) this node:
  * takes psi from the POSE quaternion (heading at rest; atan2(vy,vx) is
    undefined when the cart is stopped, so we do NOT use it),
  * rotates the body-frame twist into the map frame by psi,
  * attaches (length, radius) from the yaml,
and publishes them all as a CapsuleObstacleArray in the map frame.

It publishes EVERY tracked obstacle. It does not pad or pick the closest n --
the MPC does that.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tt_interfaces.msg import CapsuleObstacle, CapsuleObstacleArray


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class CapsuleObstacleManager(Node):
    def __init__(self):
        super().__init__('capsule_obstacle_manager')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('publish_topic', '/capsule_obstacles')
        self.declare_parameter('obstacle_ids', [0])

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        pub_topic = self.get_parameter('publish_topic').value
        ids = list(self.get_parameter('obstacle_ids').value)

        self.specs = {}       # id -> dict(length, radius)
        self.latest = {}      # id -> dict(x,y,psi,vx,vy) or None
        for oid in ids:
            self.declare_parameter(f'obstacle_{oid}.topic', f'/human{oid+1}/odom_smoothed')
            self.declare_parameter(f'obstacle_{oid}.length', 0.50)
            self.declare_parameter(f'obstacle_{oid}.radius', 0.20)
            topic = self.get_parameter(f'obstacle_{oid}.topic').value
            self.specs[oid] = dict(
                length=float(self.get_parameter(f'obstacle_{oid}.length').value),
                radius=float(self.get_parameter(f'obstacle_{oid}.radius').value))
            self.latest[oid] = None
            self.create_subscription(
                Odometry, topic, self._make_cb(oid), 10)
            self.get_logger().info(f"obstacle {oid} <- {topic} "
                                   f"(L={self.specs[oid]['length']}, R={self.specs[oid]['radius']})")

        self.pub = self.create_publisher(CapsuleObstacleArray, pub_topic, 10)
        self.create_timer(1.0 / self.rate_hz, self._publish)

    def _make_cb(self, oid):
        def cb(msg: Odometry):
            psi = yaw_from_quat(msg.pose.pose.orientation)
            # twist is expressed in the child frame (body); rotate into map by psi
            vbx = msg.twist.twist.linear.x
            vby = msg.twist.twist.linear.y
            c, s = math.cos(psi), math.sin(psi)
            self.latest[oid] = dict(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                psi=psi,
                vx=c * vbx - s * vby,
                vy=s * vbx + c * vby,
            )
        return cb

    def _publish(self):
        arr = CapsuleObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        for oid, st in self.latest.items():
            if st is None:
                continue
            m = CapsuleObstacle()
            m.id = int(oid)
            m.x = st['x']; m.y = st['y']; m.psi = st['psi']
            m.vx = st['vx']; m.vy = st['vy']
            m.length = self.specs[oid]['length']
            m.radius = self.specs[oid]['radius']
            arr.obstacles.append(m)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CapsuleObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()