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
from tutorial_5.tsid_wrapper import TSIDWrapper
import tutorial_5.config as conf

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
        self.State="right"
        self.push_state_start_time=None

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

    def pushing_force_state_machine(self,t,t_period,t_push,force_magnitude):
        #pushing force first from the right, then from the left and finally from the back
        #state: left, right, back, finsihed

        # initialize start time on first call
        if self.push_state_start_time is None:
            self.push_state_start_time = t

        elapsed = t - self.push_state_start_time


        directions={
            "right": np.array([0.0 , 1.0, 0.0]),
            "left": np.array([0.0 , -1.0, 0.0]),
            "back": np.array([1.0 , 0.0, 0.0]),
            
        }
        force=np.zeros(3)
        if elapsed <t_period:
            force=np.zeros(3)
            
        elif self.State=="finished":
            return self.State, force#finished


        elif elapsed < (t_period +t_push):
            force=force_magnitude* directions[self.State]
        else:
            next_state= {
                "right": "left",
                "left": "back",
                "back": "finished",
            }
            self.State=next_state[self.State]
            self.push_state_start_time=t  # reset timer for new State
            force=np.zeros(3)
        self.applyForce(f_w=[force[0],force[1],force[2]])


        
 
        self.applyForce(force)
        
        return self.State,force


################################################################################
# main
################################################################################


def estimate_zmp():
    pass
def estimate_cmp():
    pass
def estimate_cp():
    pass

def main(): 
    rclpy.init()
    node = rclpy.create_node('tutorial_4_standing_node')
    tsid_wrapper=TSIDWrapper(conf)
    simulator=PybulletWrapper(sim_rate=1000)
    # Setup pybullet camera
    pb.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=90,
        cameraPitch=-20,
        cameraTargetPosition=[0.0, 0.0, 0.8])
    q_home=conf.q_home


    model=tsid_wrapper.model
    urdf= conf.urdf
    #"src/Tutorial_2/talos_description/robots/talos_reduced_no_hands.urdf"
    robot=Talos(node=node,simulator=simulator,urdf=urdf,model=model,q=q_home,useFixedBase=False)
        

    
    t_publish = 0.0
    push_state = "default"
    force_magnitude=10

    pb.enableJointForceTorqueSensor(robot.id(), robot.jointNameIndexMap()["leg_right_6_joint"], True)
    pb.enableJointForceTorqueSensor(robot.id(), robot.jointNameIndexMap()["leg_left_6_joint"], True)

   



    while rclpy.ok():

        # elaped time
        t = simulator.simTime()

        simulator.step()
        simulator.debug()
        robot.update()
        rclpy.spin_once(node,timeout_sec=0)
        q=robot.q()
        v=robot.v()
        t_period=3
        t_push=1

        wren = pb.getJointState(robot.id(), robot.jointNameIndexMap()["leg_right_6_joint"])[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren
        [5]])
        wr_rankle = pin.Force(wnp)
        wren = pb.getJointState(robot.id(), robot.jointNameIndexMap()[
        "leg_left_6_joint"])[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren
        [5]])
        wl_lankle = pin.Force(wnp)

        data = robot._model.createData()
        pin.framesForwardKinematics(robot._model, data, q)
        H_w_lsole = data.oMf[robot._model.getFrameId("left_sole_link")]
        H_w_rsole = data.oMf[robot._model.getFrameId("right_sole_link")]
        H_w_lankle = data.oMf[robot._model.getFrameId("leg_left_6_joint")]
        H_w_rankle = data.oMf[robot._model.getFrameId("leg_right_6_joint")]

        if push_state != "finished":
            push_state,force=robot.pushing_force_state_machine(t,t_period,t_push,force_magnitude)
   
        tau_sol,dv_sol= tsid_wrapper.update(q,v,t)
  

        # TODO: command to the robot
        robot.setActuatedJointTorques(tau_sol)




        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            T_frame_w, v_frame_w=tsid_wrapper.baseState()
            robot.publish(T_frame_w)

            # TODO: publish current state
    
if __name__ == '__main__': 
    main()
    
