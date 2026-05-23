#!/usr/bin/env python3


import sys

import rclpy

from rclpy.node import Node

from interactive_markers import InteractiveMarkerServer

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from tf2_ros import TransformListener, Buffer

from geometry_msgs.msg import PoseStamped



class InteractiveMarkerNode(Node):


    def __init__(self):

        super().__init__('simple_marker')

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.marker_pos_pub = self.create_publisher(PoseStamped, 'marker_pose', 10)

        # Interactive marker server
        self.server = InteractiveMarkerServer(self, 'simple_marker')

        # Wait for TF to become available before spawning
        self.create_timer(1.0, self._init_marker)
        self._spawned = False


    def _init_marker(self):
        if self._spawned:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'arm_right_7_joint',
                rclpy.time.Time()

            )
        except Exception as e:
            self.get_logger().info(f'TF not available yet: {e}')
            return

        self._spawn_marker(transform)
        self._spawned = True
        self.get_logger().info('Marker spawned at hand position!')


    def _spawn_marker(self, transform):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = 'my_marker'
        int_marker.description = 'Simple 6-DOF Control'

        # Spawn at hand position
        t = transform.transform
        int_marker.pose.position.x = t.translation.x
        int_marker.pose.position.y = t.translation.y
        int_marker.pose.position.z = t.translation.z
        int_marker.pose.orientation = t.rotation

        # Visible box
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # 6DOF controls
        for name, mode, x, y, z in [
            ('move_x',   InteractiveMarkerControl.MOVE_AXIS,   1, 0, 0),
            ('move_y',   InteractiveMarkerControl.MOVE_AXIS,   0, 1, 0),
            ('move_z',   InteractiveMarkerControl.MOVE_AXIS,   0, 0, 1),
            ('rotate_x', InteractiveMarkerControl.ROTATE_AXIS, 1, 0, 0),
            ('rotate_y', InteractiveMarkerControl.ROTATE_AXIS, 0, 1, 0),
            ('rotate_z', InteractiveMarkerControl.ROTATE_AXIS, 0, 0, 1),

        ]:

            control = InteractiveMarkerControl()
            control.name = name
            control.interaction_mode = mode
            control.orientation.x = float(x)
            control.orientation.y = float(y)
            control.orientation.z = float(z)
            control.orientation.w = 1.0
            int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self._feedback_callback)
        self.server.applyChanges()


    def _feedback_callback(self, feedback):

        msg = PoseStamped()
        msg.header = feedback.header
        msg.pose = feedback.pose
        self.marker_pos_pub.publish(msg)

        p = feedback.pose.position
        self.get_logger().info(f'{feedback.marker_name} is now at {p.x:.2f}, {p.y:.2f}, {p.z:.2f}')


def main():

    rclpy.init(args=sys.argv)

    node = InteractiveMarkerNode()

    rclpy.spin(node)

    node.server.shutdown()



if __name__ == '__main__':

    main()