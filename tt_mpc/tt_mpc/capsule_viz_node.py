#!/usr/bin/env python3
"""
capsule_viz_node.py  --  current-pose markers for rviz.

Draws the truck, trailer and obstacle capsules as stadium (LINE_STRIP) outlines
from the CURRENT state/obstacles, plus the virtual-wall box. Predicted
trajectories are NOT drawn here -- those are the MPC's nav_msgs/Path topics
(/mpc/pred_trailer, /mpc/pred_truck, /mpc/pred_obstacle) shown with rviz's
native Path display. Keeping marker work off the control thread.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tt_interfaces.msg import CapsuleObstacleArray

from . import capsule_geometry as geom


def _line_strip(ns, mid, frame, pts, rgba, width=0.02):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    for x, y in pts:
        p = Point(); p.x = float(x); p.y = float(y); p.z = 0.0
        m.points.append(p)
    return m


def _sphere(ns, mid, frame, pt, rgba, d=0.05):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = d
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    m.pose.position.x = float(pt[0]); m.pose.position.y = float(pt[1]); m.pose.position.z = 0.0
    return m


def _text(ns, mid, frame, pt, text, rgba=(0.85, 0.85, 0.85, 0.9), h=0.10):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = h
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    m.pose.position.x = float(pt[0]); m.pose.position.y = float(pt[1]); m.pose.position.z = 0.18
    m.text = text
    return m


class CapsuleVizNode(Node):
    def __init__(self):
        super().__init__('capsule_viz_node')
        d = self.declare_parameter
        d('rate_hz', 15.0)
        d('marker_frame', 'map')
        d('state_topic', '/truck_trailer/state')
        d('obstacles_topic', '/capsule_obstacles')
        d('marker_topic', '/capsule_markers')
        # shared geometry (/** block)
        d('kinematics.L0', 0.42); d('kinematics.M0', -0.02); d('kinematics.L1', 0.537)
        d('footprint.tractor.length', 0.55); d('footprint.tractor.width', 0.35); d('footprint.tractor.offset', 0.215)
        d('footprint.trailer.length', 0.55); d('footprint.trailer.width', 0.35); d('footprint.trailer.offset', 0.22)
        d('virtual_walls.enabled', True)
        d('virtual_walls.x_min', -0.9); d('virtual_walls.x_max', 2.65)
        d('virtual_walls.y_min', -1.25); d('virtual_walls.y_max', 4.9)

        g = lambda n: self.get_parameter(n).value
        self.frame = g('marker_frame')
        self.L0, self.M0, self.L1 = g('kinematics.L0'), g('kinematics.M0'), g('kinematics.L1')
        self.tr = dict(L=g('footprint.tractor.length'), W=g('footprint.tractor.width'), off=g('footprint.tractor.offset'))
        self.tl = dict(L=g('footprint.trailer.length'), W=g('footprint.trailer.width'), off=g('footprint.trailer.offset'))
        self.walls = dict(enabled=bool(g('virtual_walls.enabled')),
                          x_min=g('virtual_walls.x_min'), x_max=g('virtual_walls.x_max'),
                          y_min=g('virtual_walls.y_min'), y_max=g('virtual_walls.y_max'))

        self.state = None
        self.obstacles = []
        self.create_subscription(Float64MultiArray, g('state_topic'), self._on_state, 10)
        self.create_subscription(CapsuleObstacleArray, g('obstacles_topic'), self._on_obs, 10)
        self.pub = self.create_publisher(MarkerArray, g('marker_topic'), 1)
        self.create_timer(1.0 / float(g('rate_hz')), self._draw)

    def _on_state(self, msg):
        if len(msg.data) >= 4:
            self.state = np.array(msg.data[:4], float)

    def _on_obs(self, msg):
        self.obstacles = [(o.x, o.y, o.psi, o.length, o.radius) for o in msg.obstacles]

    def _draw(self):
        arr = MarkerArray()

        if self.walls['enabled']:
            box = geom.box_outline(self.walls['x_min'], self.walls['x_max'],
                                   self.walls['y_min'], self.walls['y_max'])
            arr.markers.append(_line_strip('walls', 0, self.frame, box, (0.5, 0.5, 0.5, 0.8), 0.03))

        if self.state is not None:
            pT, qT, _, _ = geom.tractor_capsule(self.state, self.L0, self.M0, self.L1,
                                                self.tr['off'], self.tr['L'])
            pTr, qTr, _, _ = geom.trailer_capsule(self.state, self.tl['off'], self.tl['L'])
            arr.markers.append(_line_strip('truck', 0, self.frame,
                                           geom.stadium_outline(pT, qT, self.tr['W'] / 2.0),
                                           (0.1, 0.4, 0.9, 0.9)))
            arr.markers.append(_line_strip('trailer', 0, self.frame,
                                           geom.stadium_outline(pTr, qTr, self.tl['W'] / 2.0),
                                           (0.1, 0.7, 0.3, 0.9)))

            # --- skeleton layer: footprint rectangles + points of interest ---
            sk = geom.skeleton_points(self.state, self.L0, self.M0, self.L1,
                                      self.tr['off'], self.tl['off'])
            arr.markers.append(_line_strip('truck_footprint', 0, self.frame,
                geom.rect_outline(sk['tractor_center'], self.state[2], self.tr['L'], self.tr['W']),
                (0.1, 0.4, 0.9, 0.9), 0.015))
            arr.markers.append(_line_strip('trailer_footprint', 0, self.frame,
                geom.rect_outline(sk['trailer_center'], self.state[3], self.tl['L'], self.tl['W']),
                (0.1, 0.7, 0.3, 0.9), 0.015))
            poi = [('trailer_axle', (0.40, 0.70, 0.20, 1.0)),
                   ('hitch',        (0.95, 0.62, 0.15, 1.0)),
                   ('truck_axle',   (0.22, 0.54, 0.87, 1.0)),
                   ('front_axle',   (0.50, 0.47, 0.87, 1.0))]
            for i, (key, rgba) in enumerate(poi):
                arr.markers.append(_sphere('poi', i, self.frame, sk[key], rgba, 0.05))
                arr.markers.append(_text('poi_labels', i, self.frame, sk[key], key, h=0.08))
            for i, key in enumerate(('tractor_center', 'trailer_center')):
                arr.markers.append(_sphere('centers', i, self.frame, sk[key], (0.7, 0.7, 0.7, 0.9), 0.035))

        for i, (x, y, psi, length, radius) in enumerate(self.obstacles):
            pO, qO = geom.obstacle_capsule(x, y, psi, length)
            arr.markers.append(_line_strip('obstacle', i, self.frame,
                                           geom.stadium_outline(pO, qO, radius),
                                           (0.9, 0.2, 0.2, 0.9)))

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CapsuleVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()