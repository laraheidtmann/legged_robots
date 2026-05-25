#!/usr/bin/env python3
"""
Tutorial 1.1 - Exercise 3: Wrench

Build and visualize a moving cage using Pinocchio SE(3) transforms.
This node extends the Exercise 1 cage with wrench coordinate transformations.

The node publishes:
- TF frames for the world, cage origin, and cage vertices
- A line marker for the cage edges
- A point expressed in a vertex frame and the same point transformed to world
- Wrenches transformed manually and with Pinocchio for comparison

Pinocchio Force convention:
    force.vector = [linear_force_x, linear_force_y, linear_force_z,
                    torque_x,       torque_y,       torque_z]
"""

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped, WrenchStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class PinocchioCageWrenchNode(Node):
    """ROS 2 node for visualizing SE(3) transforms and wrench transformations."""

    def __init__(self):
        super().__init__("pinocchio_cage_wrench_node")
        self.get_logger().info("Pinocchio cage wrench node started.")

        # ---------------------------------------------------------------------
        # Frame names
        # ---------------------------------------------------------------------
        self.world_frame = "world"
        self.cage_origin_frame = "cage_origin"
        self.vertex_frame_prefix = "cage_vertex"

        # ---------------------------------------------------------------------
        # Simulation/update settings
        # ---------------------------------------------------------------------
        self.dt = 0.05
        self.angular_velocity_world = np.array([0.0, 0.0, 0.5])
        self.linear_velocity_world = np.array([0.01, 0.002, 0.0])

        # Initial cage pose in the world frame.
        # The cage is intentionally rotated so it is not aligned with the world.
        initial_rotation = self.rotation_z(np.pi / 4.0) @ self.rotation_y(np.pi / 6.0)
        initial_translation = np.array([1.0, 0.0, 0.0])
        self.T_world_cage = pin.SE3(initial_rotation, initial_translation)

        # Static transforms from cage origin to cage vertices.
        self.T_cage_vertices = self.build_cage_vertex_transforms()

        # Edge list for drawing the cage.
        self.cage_edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # Cage/point visualization topics
        self.cage_marker_pub = self.create_publisher(
            Marker, "visualization/cage_edges", 10
        )
        self.local_point_marker_pub = self.create_publisher(
            Marker, "visualization/local_point_in_vertex_frame", 10
        )
        self.world_point_marker_pub = self.create_publisher(
            Marker, "visualization/point_transformed_to_world", 10
        )
        self.vertex_label_pub = self.create_publisher(
            Marker, "visualization/vertex_labels", 10
        )

        # Wrench topics for Exercise 3
        self.corner_wrench_pub = self.create_publisher(
            WrenchStamped, "wrenches/corner_0_original", 10
        )
        self.world_wrench_manual_pub = self.create_publisher(
            WrenchStamped, "wrenches/corner_0_to_world_manual", 10
        )
        self.world_wrench_pinocchio_pub = self.create_publisher(
            WrenchStamped, "wrenches/corner_0_to_world_pinocchio", 10
        )
        self.world_wrench_pub = self.create_publisher(
            WrenchStamped, "wrenches/world_original", 10
        )
        self.corner_wrench_manual_pub = self.create_publisher(
            WrenchStamped, "wrenches/world_to_corner_6_manual", 10
        )
        self.corner_wrench_pinocchio_pub = self.create_publisher(
            WrenchStamped, "wrenches/world_to_corner_6_pinocchio", 10
        )

        self.timer = self.create_timer(self.dt, self.publish_visualization)

    # -------------------------------------------------------------------------
    # Cage construction
    # -------------------------------------------------------------------------

    def build_cage_vertex_transforms(self):
        """Create SE(3) transforms from cage origin to each cage vertex."""
        length = 1.0
        width = 0.6
        height = 0.8

        x = length / 2.0
        y = width / 2.0
        z = height

        vertex_positions = [
            np.array([-x, -y, 0.0]),
            np.array([ x, -y, 0.0]),
            np.array([ x,  y, 0.0]),
            np.array([-x,  y, 0.0]),
            np.array([-x, -y, z]),
            np.array([ x, -y, z]),
            np.array([ x,  y, z]),
            np.array([-x,  y, z]),
        ]

        yaw_angles = self.compute_vertex_yaw_angles(vertex_positions)

        transforms = []

        for i, (position, yaw) in enumerate(zip(vertex_positions, yaw_angles)):
            R_cage_vertex = self.rotation_z(yaw)
            # Rotate top layer frames around negative x-axis
            if i >= 4:
                R_cage_vertex = R_cage_vertex @ self.rotation_x(-np.pi / 2.0)
            transforms.append(pin.SE3(R_cage_vertex, position))

        return transforms

    @staticmethod
    def compute_vertex_yaw_angles(vertex_positions):
        """Orient each vertex frame so its x-axis points toward the next corner."""
        yaw_angles = []

        for i, position in enumerate(vertex_positions):
            layer_start = (i // 4) * 4
            local_index = i % 4
            next_index = layer_start + ((local_index + 1) % 4)

            edge_direction = vertex_positions[next_index] - position
            yaw_angles.append(np.arctan2(edge_direction[1], edge_direction[0]))

        return np.array(yaw_angles)

    # -------------------------------------------------------------------------
    # Motion integration
    # -------------------------------------------------------------------------

    def integrate_world_motion_manual(self):
        """
        Integrate cage motion using matrix formulas.

        The velocity is expressed in the world frame, so the incremental rotation
        pre-multiplies the current cage pose.
        """
        omega = self.angular_velocity_world
        linear_velocity = self.linear_velocity_world

        omega_skew = self.skew(omega)
        identity = np.eye(3)

        # Cayley approximation of exp(omega_skew * dt)
        dR = np.linalg.inv(identity - 0.5 * self.dt * omega_skew) @ (
            identity + 0.5 * self.dt * omega_skew
        )

        R_new = dR @ self.T_world_cage.rotation
        t_new = self.T_world_cage.translation + self.dt * linear_velocity

        self.T_world_cage = pin.SE3(R_new, t_new)

    def integrate_world_motion_exp6(self):
        """
        Integrate cage motion using Pinocchio exp6.

        Pinocchio Motion uses [linear, angular] ordering.
        Since the velocity is expressed in the world frame, dT pre-multiplies
        the current cage pose: T_new = dT * T_old.
        """
        velocity = np.hstack((self.linear_velocity_world, self.angular_velocity_world))
        dT = pin.exp6(pin.Motion(velocity * self.dt))
        self.T_world_cage = dT * self.T_world_cage

    # -------------------------------------------------------------------------
    # Wrench transformations
    # -------------------------------------------------------------------------

    @staticmethod
    def transform_wrench_manual(wrench_in_source_frame, T_target_source):
        """
        Transform a wrench from source frame to target frame.

        T_target_source maps coordinates from source to target.

        Wrench convention:
            W = [force, torque]

        For T_target_source = (R, p):
            force_target  = R * force_source
            torque_target = R * torque_source + p x force_target

        This is the dual transform to the twist transform.
        """
        wrench = np.asarray(wrench_in_source_frame, dtype=float)

        source_force = wrench[0:3]
        source_torque = wrench[3:6]

        R = T_target_source.rotation
        p = T_target_source.translation

        target_force = R @ source_force
        target_torque = R @ source_torque + np.cross(p, target_force)

        return np.hstack((target_force, target_torque))

    def transform_wrench_to_world_manual(self, vertex_index, wrench_in_vertex_frame):
        """Transform a wrench from cage_vertex_i to world using the manual formula."""
        T_world_vertex = self.T_world_cage * self.T_cage_vertices[vertex_index]
        return self.transform_wrench_manual(wrench_in_vertex_frame, T_world_vertex)

    def transform_wrench_to_vertex_manual(self, vertex_index, wrench_in_world_frame):
        """Transform a wrench from world to cage_vertex_i using the manual formula."""
        T_world_vertex = self.T_world_cage * self.T_cage_vertices[vertex_index]
        T_vertex_world = T_world_vertex.inverse()
        return self.transform_wrench_manual(wrench_in_world_frame, T_vertex_world)

    def transform_wrench_to_world_pinocchio(self, vertex_index, wrench_in_vertex_frame):
        """Transform a wrench from cage_vertex_i to world using Pinocchio."""
        T_world_vertex = self.T_world_cage * self.T_cage_vertices[vertex_index]

        vertex_force = pin.Force(np.asarray(wrench_in_vertex_frame, dtype=float))
        world_force = T_world_vertex.act(vertex_force)

        return world_force.vector

    def transform_wrench_to_vertex_pinocchio(self, vertex_index, wrench_in_world_frame):
        """Transform a wrench from world to cage_vertex_i using Pinocchio."""
        T_world_vertex = self.T_world_cage * self.T_cage_vertices[vertex_index]

        world_force = pin.Force(np.asarray(wrench_in_world_frame, dtype=float))
        vertex_force = T_world_vertex.actInv(world_force)

        return vertex_force.vector

    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------

    def publish_visualization(self):
        """Update the cage pose and publish TFs, markers, and wrenches."""
        self.integrate_world_motion_manual()
        # To compare with Pinocchio integration, comment the line above and use:
        # self.integrate_world_motion_exp6()

        stamp = self.get_clock().now().to_msg()

        self.publish_cage_origin_tf(stamp)
        self.publish_vertex_tfs(stamp)
        self.publish_cage_edges_marker(stamp)
        self.publish_vertex_labels(stamp)
        self.publish_point_markers(stamp)
        self.publish_wrenches(stamp)

    def publish_wrenches(self, stamp):
        """
        Publish the wrench examples requested in Exercise 3.

        Example A:
            Define a wrench in corner 0, then transform it to world.

        Example B:
            Define a wrench in world, then transform it to corner 6.

        Both are published twice after transformation:
            - once with the manual formula
            - once with Pinocchio act/actInv
        """
        corner_0 = 0
        corner_6 = 6

        # Pinocchio Force convention: [force, torque]
        # Neither the force nor torque part is zero.
        wrench_corner_0 = np.array([4.0, 0.5, 0.0, 0.0, 1.0, 0.2])

        self.publish_wrench(
            stamp=stamp,
            wrench=wrench_corner_0,
            frame_id=f"{self.vertex_frame_prefix}_{corner_0}",
            publisher=self.corner_wrench_pub,
        )

        wrench_world_manual = self.transform_wrench_to_world_manual(
            corner_0, wrench_corner_0
        )
        wrench_world_pinocchio = self.transform_wrench_to_world_pinocchio(
            corner_0, wrench_corner_0
        )

        self.publish_wrench(
            stamp=stamp,
            wrench=wrench_world_manual,
            frame_id=self.world_frame,
            publisher=self.world_wrench_manual_pub,
        )
        self.publish_wrench(
            stamp=stamp,
            wrench=wrench_world_pinocchio,
            frame_id=self.world_frame,
            publisher=self.world_wrench_pinocchio_pub,
        )

        # Define a different wrench directly in the world frame.
        wrench_world = np.array([1.0, 2.0, 0.5, 0.3, 0.0, 1.0])

        self.publish_wrench(
            stamp=stamp,
            wrench=wrench_world,
            frame_id=self.world_frame,
            publisher=self.world_wrench_pub,
        )

        wrench_corner_6_manual = self.transform_wrench_to_vertex_manual(
            corner_6, wrench_world
        )
        wrench_corner_6_pinocchio = self.transform_wrench_to_vertex_pinocchio(
            corner_6, wrench_world
        )

        self.publish_wrench(
            stamp=stamp,
            wrench=wrench_corner_6_manual,
            frame_id=f"{self.vertex_frame_prefix}_{corner_6}",
            publisher=self.corner_wrench_manual_pub,
        )
        self.publish_wrench(
            stamp=stamp,
            wrench=wrench_corner_6_pinocchio,
            frame_id=f"{self.vertex_frame_prefix}_{corner_6}",
            publisher=self.corner_wrench_pinocchio_pub,
        )

    @staticmethod
    def publish_wrench(stamp, wrench, frame_id, publisher):
        """Publish a six-dimensional wrench as a ROS WrenchStamped message."""
        wrench = np.asarray(wrench, dtype=float)

        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.wrench.force.x = float(wrench[0])
        msg.wrench.force.y = float(wrench[1])
        msg.wrench.force.z = float(wrench[2])

        msg.wrench.torque.x = float(wrench[3])
        msg.wrench.torque.y = float(wrench[4])
        msg.wrench.torque.z = float(wrench[5])

        publisher.publish(msg)

    def publish_point_markers(self, stamp):
        """Publish a point in a vertex frame and the same point transformed to world."""
        vertex_index = 0
        point_in_vertex_frame = np.array([0.5, 0.0, 0.0])

        self.publish_local_point_marker(stamp, vertex_index, point_in_vertex_frame)
        self.publish_world_point_marker(stamp, vertex_index, point_in_vertex_frame)

    def publish_cage_origin_tf(self, stamp):
        """Publish T_world_cage_origin."""
        self.publish_transform(
            stamp=stamp,
            parent_frame=self.world_frame,
            child_frame=self.cage_origin_frame,
            transform=self.T_world_cage,
        )

    def publish_vertex_tfs(self, stamp):
        """Publish T_cage_origin_vertex_i for each cage corner."""
        for i, T_cage_vertex in enumerate(self.T_cage_vertices):
            self.publish_transform(
                stamp=stamp,
                parent_frame=self.cage_origin_frame,
                child_frame=f"{self.vertex_frame_prefix}_{i}",
                transform=T_cage_vertex,
            )

    def publish_transform(self, stamp, parent_frame, child_frame, transform):
        """Convert a Pinocchio SE(3) transform to a ROS TransformStamped and broadcast it."""
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame

        msg.transform.translation.x = float(transform.translation[0])
        msg.transform.translation.y = float(transform.translation[1])
        msg.transform.translation.z = float(transform.translation[2])

        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(transform.rotation)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(msg)

    def publish_cage_edges_marker(self, stamp):
        """Draw cage edges as a LINE_LIST marker in the world frame."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = "cage_edges"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0

        for start_index, end_index in self.cage_edges:
            p_start_world = (self.T_world_cage * self.T_cage_vertices[start_index]).translation
            p_end_world = (self.T_world_cage * self.T_cage_vertices[end_index]).translation
            marker.points.append(self.to_ros_point(p_start_world))
            marker.points.append(self.to_ros_point(p_end_world))

        self.cage_marker_pub.publish(marker)

    def publish_local_point_marker(self, stamp, vertex_index, point_in_vertex_frame):
        """Publish c_p directly in a cage vertex frame."""
        marker = self.create_cube_marker(
            stamp=stamp,
            frame_id=f"{self.vertex_frame_prefix}_{vertex_index}",
            namespace="local_point_in_vertex_frame",
            marker_id=vertex_index,
            size=0.10,
        )

        marker.pose.position.x = float(point_in_vertex_frame[0])
        marker.pose.position.y = float(point_in_vertex_frame[1])
        marker.pose.position.z = float(point_in_vertex_frame[2])
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.local_point_marker_pub.publish(marker)

    def publish_world_point_marker(self, stamp, vertex_index, point_in_vertex_frame):
        """Transform c_p to w_p and publish it in the world frame."""
        T_world_vertex = self.T_world_cage * self.T_cage_vertices[vertex_index]
        point_in_world_frame = T_world_vertex * point_in_vertex_frame

        marker = self.create_cube_marker(
            stamp=stamp,
            frame_id=self.world_frame,
            namespace="point_transformed_to_world",
            marker_id=vertex_index,
            size=0.10,
        )

        marker.pose.position.x = float(point_in_world_frame[0])
        marker.pose.position.y = float(point_in_world_frame[1])
        marker.pose.position.z = float(point_in_world_frame[2])
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        self.world_point_marker_pub.publish(marker)

    def publish_vertex_labels(self, stamp):
        """Publish text labels for cage vertex indices."""
        for i, T_cage_vertex in enumerate(self.T_cage_vertices):
            T_world_vertex = self.T_world_cage * T_cage_vertex

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.world_frame
            marker.ns = "vertex_labels"
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

            self.vertex_label_pub.publish(marker)

    @staticmethod
    def create_cube_marker(stamp, frame_id, namespace, marker_id, size):
        """Create a basic cube marker with position/color filled later."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        return marker

    # -------------------------------------------------------------------------
    # Math helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def skew(vector):
        """Return the skew-symmetric matrix of a 3D vector."""
        x, y, z = vector
        return np.array([
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ])

    @staticmethod
    def rotation_x(theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ])

    @staticmethod
    def rotation_y(theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ])

    @staticmethod
    def rotation_z(theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ])

    @staticmethod
    def to_ros_point(vector):
        point = Point()
        point.x = float(vector[0])
        point.y = float(vector[1])
        point.z = float(vector[2])
        return point

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)

        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
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
    node = PinocchioCageWrenchNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
