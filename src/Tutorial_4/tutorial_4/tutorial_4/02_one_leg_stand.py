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
        super().update()
        
    
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
    node = rclpy.create_node('tutorial_4_one_leg_stand_node')
    tsid_wrapper=TSIDWrapper(conf)
    simulator=PybulletWrapper(sim_rate=1000)
    q_home=conf.q_home
    model=tsid_wrapper.model
    urdf= conf.urdf
    ROBOT=Talos(node=node,simulator=simulator,urdf=urdf,model=model,q=q_home,useFixedBase=False)
        
    # Get current COM state (to preserve the Z height)
    com_state = tsid_wrapper.comState()
    p_com = com_state.pos().copy()  # current COM position [x, y, z]

    # Get current right foot position
    T_rf = tsid_wrapper.get_placement_RF()

    # Override only XY with right foot position, keep Z (height) unchanged
    p_com[0] = T_rf.translation[0]
    p_com[1] = T_rf.translation[1]
    # p_com[2] stays the same!

    # Set the new COM reference
    tsid_wrapper.setComRefState(p_com)
        
    t_publish = 0.0
    time_elapsed=False
    z_LF=0.3

    while rclpy.ok():

        # elaped time
        t = simulator.simTime()

        simulator.step()
        simulator.debug()
        ROBOT.update()
        rclpy.spin_once(node,timeout_sec=0)
        q=ROBOT.q()
        v=ROBOT.v()
        if t>2 and time_elapsed==False:
            print("Moving foot to z: ",z_LF)
            time_elapsed=True
            tsid_wrapper.remove_contact_LF()
            pose=T_rf
            pose.translation[2]=z_LF
            tsid_wrapper.set_LF_pose_ref(pose)



        
        # TODO: update TSID controller
        tau_sol,dv_sol= tsid_wrapper.update(q,v,t)



        # TODO: command to the robot
        ROBOT.setActuatedJointTorques(tau_sol)


        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            T_frame_w, v_frame_w=tsid_wrapper.baseState()
            ROBOT.publish(T_frame_w)

            # TODO: publish current state
    
if __name__ == '__main__': 
    main()
    
