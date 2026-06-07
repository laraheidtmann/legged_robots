import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt


# pinocchio
import pinocchio as pin

# simulator
import pybullet as pb
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot and controller
from tutorial_4.tsid_wrapper import TSIDWrapper
import tutorial_4.config as conf

# ROS
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R


################################################################################
# settings
################################################################################

DO_PLOT = True

################################################################################
# Robot
################################################################################

class Talos(Robot):
    def __init__(self, node,simulator, urdf, model, q=None, verbose=True, useFixedBase=False):
        z_init=1.15
        self.node=node


        super().__init__(simulator,urdf,model, [0, 0, z_init],       # Floating base initial position
              [0,0,0,1] ,q=q ,useFixedBase=useFixedBase)
        self.joint_state_publisher=self.node.create_publisher(JointState,"joint_states",10)
        self.tf_broadcaster=TransformBroadcaster(self.node)

        # TODO call base class constructor
        # TODO add publisher
        # TODO add tf broadcaster
        pass

    def update(self):
        # TODO update base class
        pass
    
    def publish(self,T_frame_w):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.actuatedJointNames()
        msg.position = list(self.actuatedJointPosition())
        msg.velocity = list(self.actuatedJointVelocity())

        self.joint_state_publisher.publish(msg)



        tf_msg = TransformStamped()

        tf_msg.header.stamp = self.node.get_clock().now().to_msg()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "base_link"  

        # Translation
        tf_msg.transform.translation.x = float(T_frame_w.translation[0])
        tf_msg.transform.translation.y = float(T_frame_w.translation[1])
        tf_msg.transform.translation.z = float(T_frame_w.translation[2])

        # Rotation matrix -> quaternion
        quat = R.from_matrix(T_frame_w.rotation).as_quat()
        # scipy gives [x, y, z, w]

        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(tf_msg)

        # TODO publish jointstate
        # TODO broadcast transformation T_b_w
        pass

################################################################################
# main
################################################################################

def main(): 
    rclpy.init()
    node = rclpy.create_node('tutorial_4_standing_node')
    tsid_wrapper=TSIDWrapper(conf)
    simulator=PybulletWrapper(sim_rate=1000)
    q_home=conf.q_home
    model=tsid_wrapper.model
    urdf= "src/Tutorial_2/talos_description/robots/talos_reduced_no_hands.urdf"
    ROBOT=Talos(node=node,simulator=simulator,urdf=urdf,model=model,q=q_home,useFixedBase=False)
        

    # TODO init TSIDWrapper
    # TODO init Simulator
    # TODO init ROBOT
    
    t_publish = 0.0

    while rclpy.ok():

        # elaped time
        t = simulator.simTime()

        # TODO: update the simulator and the robot
        simulator.step()
        simulator.debug()
        ROBOT.update()
        rclpy.spin_once(node,timeout_sec=0)
        q=ROBOT.q()
        v=ROBOT.v()
        
        # TODO: update TSID controller
        tau_sol,dv_sol= tsid_wrapper.update(q,v,t)



        # TODO: command to the robot

        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            T_frame_w, v_frame_w=tsid_wrapper.baseState()
            ROBOT.publish(T_frame_w)

            # TODO: publish current state
    
if __name__ == '__main__': 
    main()
    
