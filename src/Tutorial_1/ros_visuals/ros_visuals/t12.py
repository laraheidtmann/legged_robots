#!/usr/bin/env python3
"""
Tutorial 1.1 - Exercise 2: Twist transformations

This node extends the SE(3) cage from t11.py and publishes example spatial
velocities (twists) expressed in different coordinate frames.

Pinocchio Motion vector convention used here:
    [vx, vy, vz, wx, wy, wz]
where v is the linear part and w is the angular part.
"""

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped, TwistStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


class TwistCageNode(Node):
    """Publish a moving cage and compare manual vs Pinocchio twist transforms."""

    def __init__(self):
        super().__init__("twist_cage_node")
        self.get_logger().info("Twist cage node started.")

        # ------------------------------------------------------------------
        # Frame names
        # ------------------------------------------------------------------
        self.world_frame = "world"
        self.cage_origin_frame = "cage_origin"
        self.cage_vertex_prefix = "cage_vertex"

        # ------------------------------------------------------------------
        # Simulation parameters
        # ------------------------------------------------------------------
        self.dt = 0.05
        self.cage_angular_velocity_world = np.array([0.0, 0.0, 0.5])
        self.cage_linear_velocity_world = np.array([0.0, 0.0, 0.0])

        # Initial cage pose in the world frame. The cage is intentionally not
        # aligned with the world frame, as required by Exercise 1.
        initial_rotation = self.rotz(np.pi / 4.0) @ self.roty(np.pi / 6.0)
        initial_translation = np.array([1.0, 0.0, 0.0])
        self.T_world_cage = pin.SE3(initial_rotation, initial_translation)

        # ------------------------------------------------------------------
        # ROS publishers
        # ------------------------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cage_edges_pub = self.create_publisher(
            Marker, "visualization/cage_edges", 10
        )
        self.vertex_labels_pub = self.create_publisher(
            Marker, "visualization/cage_vertex_labels", 10
        )
        self.local_point_pub = self.create_publisher(
            Marker, "visualization/point_expressed_in_vertex_frame", 10
        )
        self.world_point_pub = self.create_publisher(
            Marker, "visualization/same_point_expressed_in_world_frame", 10
        )

        # Twist topics for Exercise 2.
        self.corner_twist_pub = self.create_publisher(
            TwistStamped, "twists/input_twist_at_vertex_0", 10
        )
        self.world_twist_manual_pub = self.create_publisher(
            TwistStamped, "twists/vertex_0_twist_to_world_manual", 10
        )
        self.world_twist_pinocchio_pub = self.create_publisher(
            TwistStamped, "twists/vertex_0_twist_to_world_pinocchio", 10
        )
        self.other_corner_twist_manual_pub = self.create_publisher(
            TwistStamped, "twists/world_twist_to_vertex_6_manual", 10
        )
        self.other_corner_twist_pinocchio_pub = self.create_publisher(
            TwistStamped, "twists/world_twist_to_vertex_6_pinocchio", 10
        )

        # ------------------------------------------------------------------
        # Cage geometry
        # ------------------------------------------------------------------
        self.cage_transforms = self.build_cage_transforms()
        self.edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        self.timer = self.create_timer(self.dt, self.publish_all)

    # ======================================================================
    # Twist transformations
    # ======================================================================

    @staticmethod
    def transform_twist_manual(T_target_source: pin.SE3, source_twist: np.ndarray) -> np.ndarray:
        """
        Express a twist from the source frame in the target frame.

        T_target_source maps coordinates from the source frame to the target
        frame. This is the same convention used by Pinocchio SE3.act(Motion).

        Formula for Motion convention [v, w]:
            w_target = R * w_source
            v_target = R * v_source + p x w_target
        """
        source_twist = np.asarray(source_twist, dtype=float)

        source_linear = source_twist[0:3]
        source_angular = source_twist[3:6]

        R = T_target_source.rotation
        p = T_target_source.translation

        target_angular = R @ source_angular
        target_linear = R @ source_linear + np.cross(p, target_angular)

        return np.hstack((target_linear, target_angular))

    def transform_vertex_twist_to_world_manual(
        self, vertex_id: int, vertex_twist: np.ndarray
    ) -> np.ndarray:
        """Transform a twist from cage_vertex_<id> to the world frame."""
        T_world_vertex = self.get_world_to_vertex_transform(vertex_id)
        return self.transform_twist_manual(T_world_vertex, vertex_twist)

    def transform_world_twist_to_vertex_manual(
        self, vertex_id: int, world_twist: np.ndarray
    ) -> np.ndarray:
        """Transform a twist from the world frame to cage_vertex_<id>."""
        T_vertex_world = self.get_world_to_vertex_transform(vertex_id).inverse()
        return self.transform_twist_manual(T_vertex_world, world_twist)

    def transform_vertex_twist_to_world_pinocchio(
        self, vertex_id: int, vertex_twist: np.ndarray
    ) -> np.ndarray:
        """Same as transform_vertex_twist_to_world_manual, but using Pinocchio."""
        T_world_vertex = self.get_world_to_vertex_transform(vertex_id)
        vertex_motion = pin.Motion(np.asarray(vertex_twist, dtype=float))
        world_motion = T_world_vertex.act(vertex_motion)
        return world_motion.vector

    def transform_world_twist_to_vertex_pinocchio(
        self, vertex_id: int, world_twist: np.ndarray
    ) -> np.ndarray:
        """Same as transform_world_twist_to_vertex_manual, but using Pinocchio."""
        T_world_vertex = self.get_world_to_vertex_transform(vertex_id)
        world_motion = pin.Motion(np.asarray(world_twist, dtype=float))
        vertex_motion = T_world_vertex.actInv(world_motion)
        return vertex_motion.vector

    # ======================================================================
    # Cage motion integration
    # ======================================================================

    def integrate_cage_motion_world_frame(self) -> None:
        """
        Move the cage using a velocity expressed in the world frame.

        Since the increment is expressed in world coordinates, it pre-multiplies
        the current pose:
            T_new = dT_world * T_old
        """
        twist_world = np.hstack(
            (self.cage_linear_velocity_world, self.cage_angular_velocity_world)
        )
        dT_world = pin.exp6(pin.Motion(twist_world * self.dt))
        self.T_world_cage = dT_world * self.T_world_cage

    # ======================================================================
    # Cage construction
    # ======================================================================

    def build_cage_transforms(self) -> list[pin.SE3]:
        """Create SE(3) transforms from cage_origin to each cage vertex."""
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

        transforms = []

        for i, (vertex_position, yaw) in enumerate(
            zip(vertices, self.compute_vertex_yaws(vertices))
        ):

            R_vertex = self.rotz(yaw)
            # Rotate top layer frames (4-7) additionally around -x by 90 deg
            if i >= 4:
                R_vertex = R_vertex @ self.rotx(-np.pi / 2.0)
            transforms.append(pin.SE3(R_vertex, vertex_position))

        return transforms

    @staticmethod
    def compute_vertex_yaws(vertices: list[np.ndarray]) -> np.ndarray:
        """Orient each vertex x-axis toward the next vertex on the same layer."""
        yaw_angles = []

        for i, vertex in enumerate(vertices):
            layer_start = (i // 4) * 4
            next_vertex_on_layer = layer_start + ((i + 1) % 4)
            direction = vertices[next_vertex_on_layer] - vertex
            yaw_angles.append(np.arctan2(direction[1], direction[0]))

        return np.array(yaw_angles)

    def get_world_to_vertex_transform(self, vertex_id: int) -> pin.SE3:
        """Return T_world_vertex for cage_vertex_<vertex_id>."""
        return self.T_world_cage * self.cage_transforms[vertex_id]

    # ======================================================================
    # Main publish loop
    # ======================================================================

    def publish_all(self) -> None:
        """Update cage pose and publish TFs, markers, and twist examples."""
        self.integrate_cage_motion_world_frame()
        stamp = self.get_clock().now().to_msg()

        self.publish_cage_origin_tf(stamp)
        self.publish_vertex_tfs(stamp)
        self.publish_cage_edges(stamp)
        self.publish_vertex_labels(stamp)
        self.publish_point_examples(stamp)
        self.publish_twist_examples(stamp)

    def publish_twist_examples(self, stamp) -> None:
        """Publish the twist transformations requested in Exercise 2."""
        vertex_1 = 0
        vertex_2 = 6

        # Example 1: define a twist in vertex_0, then express it in world.
        # Pinocchio Motion convention: [vx, vy, vz, wx, wy, wz]
        vertex_0_twist = np.array([0.5, 0.0, 0.0, 1.0, 0.0, 0.0])

        world_twist_manual = self.transform_vertex_twist_to_world_manual(
            vertex_1, vertex_0_twist
        )
        world_twist_pinocchio = self.transform_vertex_twist_to_world_pinocchio(
            vertex_1, vertex_0_twist
        )

        self.publish_twist(
            stamp,
            vertex_0_twist,
            self.vertex_frame_name(vertex_1),
            self.corner_twist_pub,
        )
        self.publish_twist(
            stamp,
            world_twist_manual,
            self.world_frame,
            self.world_twist_manual_pub,
        )
        self.publish_twist(
            stamp,
            world_twist_pinocchio,
            self.world_frame,
            self.world_twist_pinocchio_pub,
        )

        # Example 2: define a twist in world, then express it in vertex_6.
        world_twist = np.array([0.2, 0.1, 0.0, 0.0, 0.0, 1.0])

        vertex_6_twist_manual = self.transform_world_twist_to_vertex_manual(
            vertex_2, world_twist
        )
        vertex_6_twist_pinocchio = self.transform_world_twist_to_vertex_pinocchio(
            vertex_2, world_twist
        )

        self.publish_twist(
            stamp,
            vertex_6_twist_manual,
            self.vertex_frame_name(vertex_2),
            self.other_corner_twist_manual_pub,
        )
        self.publish_twist(
            stamp,
            vertex_6_twist_pinocchio,
            self.vertex_frame_name(vertex_2),
            self.other_corner_twist_pinocchio_pub,
        )

    # ======================================================================
    # ROS publishing helpers
    # ======================================================================

    def publish_cage_origin_tf(self, stamp) -> None:
        """Broadcast T_world_cage_origin."""
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

    def publish_vertex_tfs(self, stamp) -> None:
        """Broadcast T_cage_origin_vertex_i for all cage vertices."""
        for vertex_id, T_cage_vertex in enumerate(self.cage_transforms):
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.cage_origin_frame
            msg.child_frame_id = self.vertex_frame_name(vertex_id)

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

    def publish_cage_edges(self, stamp) -> None:
        """Publish a LINE_LIST marker showing the cage edges in world frame."""
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

        for start_id, end_id in self.edges:
            start_world = self.get_world_to_vertex_transform(start_id).translation
            end_world = self.get_world_to_vertex_transform(end_id).translation
            marker.points.append(self.to_point(start_world))
            marker.points.append(self.to_point(end_world))

        self.cage_edges_pub.publish(marker)

    def publish_vertex_labels(self, stamp) -> None:
        """Publish text labels next to each cage vertex."""
        for vertex_id in range(len(self.cage_transforms)):
            T_world_vertex = self.get_world_to_vertex_transform(vertex_id)

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.world_frame
            marker.ns = "cage_vertex_labels"
            marker.id = vertex_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(T_world_vertex.translation[0])
            marker.pose.position.y = float(T_world_vertex.translation[1])
            marker.pose.position.z = float(T_world_vertex.translation[2] + 0.08)
            marker.pose.orientation.w = 1.0
            marker.text = str(vertex_id)
            marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            self.vertex_labels_pub.publish(marker)

    def publish_point_examples(self, stamp) -> None:
        """Publish one point in a vertex frame and the same point in world."""
        vertex_id = 0
        point_in_vertex_frame = np.array([0.5, 0.0, 0.0])

        self.publish_local_point_marker(stamp, vertex_id, point_in_vertex_frame)
        self.publish_world_point_marker(stamp, vertex_id, point_in_vertex_frame)

    def publish_local_point_marker(
        self, stamp, vertex_id: int, point_in_vertex_frame: np.ndarray
    ) -> None:
        """Publish c_p directly in cage_vertex_<id>."""
        marker = self.make_cube_marker(
            stamp=stamp,
            frame_id=self.vertex_frame_name(vertex_id),
            namespace="point_expressed_in_vertex_frame",
            marker_id=vertex_id,
            color=(1.0, 0.0, 0.0, 1.0),
        )
        marker.pose.position.x = float(point_in_vertex_frame[0])
        marker.pose.position.y = float(point_in_vertex_frame[1])
        marker.pose.position.z = float(point_in_vertex_frame[2])

        self.local_point_pub.publish(marker)

    def publish_world_point_marker(
        self, stamp, vertex_id: int, point_in_vertex_frame: np.ndarray
    ) -> None:
        """Transform c_p into w_p and publish it in the world frame."""
        T_world_vertex = self.get_world_to_vertex_transform(vertex_id)
        point_in_world_frame = T_world_vertex * point_in_vertex_frame

        marker = self.make_cube_marker(
            stamp=stamp,
            frame_id=self.world_frame,
            namespace="same_point_expressed_in_world_frame",
            marker_id=vertex_id,
            color=(1.0, 0.0, 0.2, 1.0),
        )
        marker.pose.position.x = float(point_in_world_frame[0])
        marker.pose.position.y = float(point_in_world_frame[1])
        marker.pose.position.z = float(point_in_world_frame[2])

        self.world_point_pub.publish(marker)

    def publish_twist(
        self,
        stamp,
        twist_vector: np.ndarray,
        frame_id: str,
        publisher,
    ) -> None:
        """Publish a 6D Pinocchio-style twist as geometry_msgs/TwistStamped."""
        twist_vector = np.asarray(twist_vector, dtype=float)

        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.twist.linear.x = float(twist_vector[0])
        msg.twist.linear.y = float(twist_vector[1])
        msg.twist.linear.z = float(twist_vector[2])
        msg.twist.angular.x = float(twist_vector[3])
        msg.twist.angular.y = float(twist_vector[4])
        msg.twist.angular.z = float(twist_vector[5])

        publisher.publish(msg)

    @staticmethod
    def make_cube_marker(stamp, frame_id: str, namespace: str, marker_id: int, color) -> Marker:
        """Create a small cube marker with a fixed size and configurable color."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.pose.orientation.w = 1.0
        return marker

    def vertex_frame_name(self, vertex_id: int) -> str:
        """Return the ROS frame name for a cage vertex."""
        return f"{self.cage_vertex_prefix}_{vertex_id}"

    # ======================================================================
    # Math helpers
    # ======================================================================

    @staticmethod
    def rotx(theta: float) -> np.ndarray:
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ])

    @staticmethod
    def roty(theta: float) -> np.ndarray:
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ])

    @staticmethod
    def rotz(theta: float) -> np.ndarray:
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ])

    @staticmethod
    def to_point(vector: np.ndarray) -> Point:
        point = Point()
        point.x = float(vector[0])
        point.y = float(vector[1])
        point.z = float(vector[2])
        return point

    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
        """Convert a 3x3 rotation matrix to ROS quaternion order x, y, z, w."""
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistCageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
