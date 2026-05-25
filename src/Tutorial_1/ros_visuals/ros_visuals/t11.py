#!/usr/bin/env python3

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class PinocchioCageNode(Node):
    def __init__(self):
        super().__init__("pinocchio_cage_node")
        self.get_logger().info("Pinocchio cage node is started.")

        self.world_frame = "world"
        self.cage_origin_frame = "cage_origin"
        self.cage_frame_prefix = "cage_vertex"

        #Rotation and Translation of the entire cube relative to world frame:

        self.R = self.rotz(np.pi / 4.0) @ self.roty(np.pi / 6.0)
        self.T=np.array([1.0, 0.0, 0.0])

        self.T_world_cage = pin.SE3(
            self.R,
            self.T
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "cage_marker", 10)
        self.point_pub=self.create_publisher(Marker,"point_marker",10) #world frame
        self.point_pub_1=self.create_publisher(Marker,"point_marker_1",10)

        self.label_pub = self.create_publisher(Marker, "cage_labels", 10)

        self.cage_transforms = self.build_cage_transforms()

        self.edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        self.timer = self.create_timer(0.05, self.publish_cage)
    
    def rotate_and_translate_cage(self, angular_vel, linear_vel):
        dt = 0.1

        angular_vel = np.asarray(angular_vel, dtype=float)
        linear_vel = np.asarray(linear_vel, dtype=float)

        wx = angular_vel[0]
        wy = angular_vel[1]
        wz = angular_vel[2]

        S = np.array([
            [0.0, -wz,  wy],
            [wz,  0.0, -wx],
            [-wy, wx,  0.0],
        ])

        R = self.R
        t = self.T

        I = np.eye(3)

        # Cayley integration for rotation
        dR = np.linalg.inv(I - 0.5 * dt * S) @ (I + 0.5 * dt * S)

        R_new = dR @ R
        t_new = t + dt * linear_vel  

        self.T_world_cage = pin.SE3(R_new, t_new)

        self.R=R_new
        self.T=t_new

    def rotate_and_translate_cage_exp6(self, angular_vel, linear_vel):
        dt = 0.05

        angular_vel = np.asarray(angular_vel, dtype=float)
        linear_vel = np.asarray(linear_vel, dtype=float)

        # Pinocchio Motion convention: [linear, angular]
        twist = np.hstack((linear_vel, angular_vel))

        # Integrate over timestep
        dT = pin.exp6(pin.Motion(twist * dt))

        # Velocity expressed in cage frame:
        self.T_world_cage =  self.T_world_cage * dT

    def build_cage_transforms(self):
        length = 1.0
        width = 0.6
        height = 0.8

        x = length / 2.0
        y = width / 2.0
        z = height

        vertices = [
            np.array([-x, -y, 0.0]),
            np.array([ x, -y, 0.0]),
            np.array([ x,  y, 0.0]),
            np.array([-x,  y, 0.0]),

            np.array([-x, -y, z]),
            np.array([ x, -y, z]),
            np.array([ x,  y, z]),
            np.array([-x,  y, z]),
        ]

        yaw_angles = self.compute_yaw_angles_from_vertices(vertices)

        transforms = []
        for p, yaw in zip(vertices, yaw_angles):
            R = self.rotz(yaw)
            T = pin.SE3(R, p)
            transforms.append(T)

        return transforms

    def compute_yaw_angles_from_vertices(self, vertices):
        yaw_angles = []

        for i in range(len(vertices)):
            layer_start = (i // 4) * 4
            local_i = i % 4
            j = layer_start + ((local_i + 1) % 4)

            direction = vertices[j] - vertices[i]
            yaw = np.arctan2(direction[1], direction[0])

            yaw_angles.append(yaw)

        return np.array(yaw_angles)

    def publish_cage(self):
        self.rotate_and_translate_cage(
            angular_vel=np.array([0.0, 0.0, 0.5]),
            linear_vel=np.array([0.01, 0.002, 0.0]),
        )

        now = self.get_clock().now().to_msg()

        self.publish_cage_origin_tf(now)
        self.publish_tf_frames(now)
        self.publish_marker(now)
        self.publish_vertex_labels(now)
        self.publish_point_marker(
            now,
            vertex=0,
            point=np.array([0.5, 0.0, 0.0])
        )
        self.publish_point_marker(
            now,
            vertex=0,
            point=np.array([0.5, 0.0, 0.0])
        )
    def publish_cage_origin_tf(self, stamp):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.cage_origin_frame

        msg.transform.translation.x = float(self.T_world_cage.translation[0])
        msg.transform.translation.y = float(self.T_world_cage.translation[1])
        msg.transform.translation.z = float(self.T_world_cage.translation[2])

        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(
            self.T_world_cage.rotation
        )

        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(msg)

    def publish_tf_frames(self, stamp):
        for i, T_cage_vertex in enumerate(self.cage_transforms):
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.cage_origin_frame
            msg.child_frame_id = f"{self.cage_frame_prefix}_{i}"

            msg.transform.translation.x = float(T_cage_vertex.translation[0])
            msg.transform.translation.y = float(T_cage_vertex.translation[1])
            msg.transform.translation.z = float(T_cage_vertex.translation[2])

            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(
                T_cage_vertex.rotation
            )

            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(msg)

    def publish_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame

        marker.ns = "pinocchio_cage"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.025

        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0

        for i, j in self.edges:
            pi = (self.T_world_cage * self.cage_transforms[i]).translation
            pj = (self.T_world_cage * self.cage_transforms[j]).translation

            marker.points.append(self.to_point(pi))
            marker.points.append(self.to_point(pj))

        self.marker_pub.publish(marker)
    def publish_point_marker_1(self,stamp,vertex,point): #in the vertices coordinate frame
        point = np.asarray(point, dtype=float)

        point_marker = Marker()
        point_marker.header.stamp = stamp
        point_marker.header.frame_id = "cage_vertex_"+str(vertex)

        point_marker.ns = "point_marker_1"
        point_marker.id = vertex
        point_marker.type = Marker.CUBE
        point_marker.action = Marker.ADD

        point_marker.scale.x = 0.1
        point_marker.scale.y = 0.1
        point_marker.scale.z = 0.1

        point_marker.color.r = 1.0
        point_marker.color.g = 0.0
        point_marker.color.b = 0.0
        point_marker.color.a = 1.0

      
        point_marker.pose.position.x = float(point[0])
        point_marker.pose.position.y = float(point[1])
        point_marker.pose.position.z = float(point[2])
        point_marker.pose.orientation.w = 1.0

        self.point_pub_1.publish(point_marker)

    def publish_point_marker(self, stamp, vertex, point): #in world frame
        point = np.asarray(point, dtype=float)

        point_marker = Marker()
        point_marker.header.stamp = stamp
        point_marker.header.frame_id = self.world_frame

        point_marker.ns = "point_marker"
        point_marker.id = vertex
        point_marker.type = Marker.CUBE
        point_marker.action = Marker.ADD

        point_marker.scale.x = 0.1
        point_marker.scale.y = 0.1
        point_marker.scale.z = 0.1

        point_marker.color.r = 1.0
        point_marker.color.g = 0.0
        point_marker.color.b = 0.2
        point_marker.color.a = 1.0

        T_world_vertex = self.T_world_cage * self.cage_transforms[vertex]

        # point is expressed in the local coordinate frame of that vertex
        p_world = T_world_vertex * point

        point_marker.pose.position.x = float(p_world[0])
        point_marker.pose.position.y = float(p_world[1])
        point_marker.pose.position.z = float(p_world[2])
        point_marker.pose.orientation.w = 1.0

        self.point_pub.publish(point_marker)
    def publish_vertex_labels(self, stamp):
        for i, T_cage_vertex in enumerate(self.cage_transforms):
            T_world_vertex = self.T_world_cage * T_cage_vertex

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.world_frame

            marker.ns = "cage_labels"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = float(T_world_vertex.translation[0])
            marker.pose.position.y = float(T_world_vertex.translation[1])
            marker.pose.position.z = float(T_world_vertex.translation[2] + 0.08)
            marker.pose.orientation.w = 1.0

            marker.text = str(i)
            marker.scale.z = 0.12

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            self.label_pub.publish(marker)

    @staticmethod
    def rotz(theta):
        c = np.cos(theta)
        s = np.sin(theta)

        return np.array([
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ])

    @staticmethod
    def rotx(theta):
        c = np.cos(theta)
        s = np.sin(theta)

        return np.array([
            [1.0, 0.0, 0.0],
            [0.0,  c,  -s],
            [0.0,  s,   c],
        ])
    @staticmethod
    def roty(theta):
        c = np.cos(theta)
        s = np.sin(theta)

        return np.array([
            [ c, 0.0,  s],
            [0.0, 1.0, 0.0],
            [-s, 0.0,  c],
        ])
    @staticmethod
    def to_point(v):
        p = Point()
        p.x = float(v[0])
        p.y = float(v[1])
        p.z = float(v[2])
        return p

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        trace = np.trace(R)

        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return float(qx), float(qy), float(qz), float(qw)


def main(args=None):
    rclpy.init(args=args)
    node = PinocchioCageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()