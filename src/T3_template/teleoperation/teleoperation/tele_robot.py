#!/usr/bin/env python3


import sys

from interactive_markers import InteractiveMarkerServer
import rclpy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped


def processFeedback(feedback):
    msg=PoseStamped()
    p = feedback.pose.position

    node.marker_pos_pub.publish(msg)
    
    print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')



if __name__ == '__main__':
     main()

def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('simple_marker')
    node.tf_buffer=Buffer()
    node.tf_listener=TransformListener(node.tf_buffer,node)
    node.marker_pos_pub=node.create_publisher(PoseStamped,"marker_pose",10)


    # create an interactive marker server on the namespace simple_marker
    server = InteractiveMarkerServer(node, 'simple_marker')

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'arm_right_7_joint'
    int_marker.name = 'my_marker'
    int_marker.description = 'Simple 6-DOF Control'

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'move_x'
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    rotate_control.orientation.x=1.0
    rotate_control.orientation.w=1.0   
    int_marker.controls.append(rotate_control)
 
    
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'move_y'
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    rotate_control.orientation.y=1.0
    rotate_control.orientation.w=1.0
    int_marker.controls.append(rotate_control)


    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'move_z'
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    rotate_control.orientation.z=1.0
    rotate_control.orientation.w=1.0
    int_marker.controls.append(rotate_control)


    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'rotate_x'
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation.x=1.0
    rotate_control.orientation.w=1.0
    int_marker.controls.append(rotate_control)


    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'rotate_y'
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation.y=1.0
    rotate_control.orientation.w=1.0
    int_marker.controls.append(rotate_control)


    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'rotate_z'
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation.z=1.0
    rotate_control.orientation.w=1.0
    int_marker.controls.append(rotate_control)

    # add the control to the interactive marker
    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, feedback_callback=processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rclpy.spin(node)
    server.shutdown()