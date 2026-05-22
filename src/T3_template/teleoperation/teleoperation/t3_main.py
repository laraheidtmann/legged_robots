import numpy as np
import numpy.linalg as la

# simulator (#TODO: set your own import path!)
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# modeling
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from enum import Enum
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

################################################################################
# utility functions
################################################################################

class State(Enum):
    JOINT_SPLINE = 0,
    CART_SPLINE = 1

################################################################################
# Robot
################################################################################

class Talos(Robot):
    

    def __init__(self, simulator, q=None, verbose=True, useFixedBase=True):
           
            urdf = "src/talos_description/robots/talos_reduced.urdf"
            path_meshes = "src/talos_description/meshes/../.."
            z_init = 1.15
            q_actuated_home = np.zeros(32)
            q_actuated_home[:6] = np.array([0, 0, 0, 0, 0, 0])
            q_actuated_home[6:12] = np.array([0, 0, 0, 0, 0, 0])
            q_actuated_home[14:22] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])
            q_actuated_home[22:30] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])

            # Initialization position including floating base
            q_home = np.hstack([np.array([0, 0, z_init, 0, 0, 0, 1]), q_actuated_home])

             
            
        
            self._wrapper = pin.RobotWrapper.BuildFromURDF(urdf,                        # Model description
                                                path_meshes,                 # Model geometry descriptors 
                                                None,   # Floating base model. Use "None" if fixed
                                                True,                        # Printout model details
                                                None)                        # Load meshes different from the descripor
            
            # Get model from wrapper
            self.model= self._wrapper.model

            super().__init__(simulator,urdf,self.model, [0, 0, z_init],       # Floating base initial position
              [0,0,0,1] ,q=q_actuated_home ,useFixedBase=useFixedBase)

            self.node=rclpy.create_node("teleoperator")
            self.joint_state_publisher=self.node.create_publisher(JointState,"joint_states",10)
            self.tf_broadcaster=TransformBroadcaster(self.node)
            #TODO: Create RobotWrapper (fixed base), Call base class constructor, make publisher  
            #None
        
    def update(self):
        # TODO: update base class, update pinocchio robot wrapper's kinematics
        # Updates self._q and self._v from PyBullet

        super().update()
        # Update Pinocchio kinematics using current state

        pin.forwardKinematics(

            self._wrapper.model,
            self._wrapper.data,
            self.q(),
            self.v(),

        )


        pin.updateFramePlacements(

            self._wrapper.model,
            self._wrapper.data,

        )
 
    def publish(self):

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.actuatedJointNames()
        msg.position = list(self.actuatedJointPosition())
        msg.velocity = list(self.actuatedJointVelocity())

        self.joint_state_publisher.publish(msg)


        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = "world"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)
        
    def wrapper(self):
        return self._wrapper

    def data(self):
        return self._wrapper.data
    

################################################################################
# Controllers
################################################################################

class JointSpaceController: #takes robot gains kp and kd and has updtade taking all the reference --> compute torque
    """JointSpaceController
    Tracking controller in jointspace
    """
    def __init__(self, robot, Kp, Kd):        
        # Save gains, robot ref
        None
    
    def update(self, q_r, q_r_dot, q_r_ddot):
        self.wrapper.update()
        # Compute jointspace torque, return torque
        None
    
class CartesianSpaceController:
    """CartesianSpaceController
    Tracking controller in cartspace
    """
    def __init__(self, robot, joint_name, Kp, Kd):
        # save gains, robot ref
        None
        
    def update(self, X_r, X_dot_r, X_ddot_r):
        # compute cartesian control torque, return torque
        None

################################################################################
# Application
################################################################################
    
class Envionment:
    def __init__(self):        
        # state
        self.cur_state = State.JOINT_SPLINE
        
        # create simulation
        self.simulator = PybulletWrapper()
        
        ########################################################################
        # spawn the robot
        ########################################################################
        self.q_home = np.zeros(32)
        self.q_home[14:22] = np.array([0, +0.45, 0, -1, 0, 0, 0, 0 ])
        self.q_home[22:30] = np.array([0, -0.45, 0, -1, 0, 0, 0, 0 ])
        
        self.q_init = np.zeros(32)

        # TODO: spawn robot

        ########################################################################
        # joint space spline: init -> home
        ########################################################################

        # TODO: create a joint spline 
        # TODO: create a joint controller
        
        ########################################################################
        # cart space: hand motion
        ########################################################################

        # TODO: create a cartesian controller
        
        ########################################################################
        # logging
        ########################################################################
        
        # TODO: publish robot state every 0.01 s to ROS
        self.t_publish = 0.0
        self.publish_period = 0.01
        
    def update(self, t, dt):
        
        # TODO: update the robot and model
        # self.robot.update()

        # update the controllers
        # TODO: Do inital jointspace, switch to cartesianspace control
        
        # command the robot
        # self.robot.setActuatedJointTorques(tau)
            
        # TODO: publish ros stuff
        
        None

def main():    
    rclpy.init()

    env = Envionment()
    robot=Talos(env.simulator)

    
    # TODO: Keep looping while ros is running 
    while True:
        robot.update()
        robot.publish()
        t = env.simulator.simTime()
        dt = env.simulator.stepTime()
        
        env.update(t, dt)
        
        env.simulator.debug()
        env.simulator.step()
        
if __name__ == '__main__': 
    # TODO: Todo init node
    main()
    
