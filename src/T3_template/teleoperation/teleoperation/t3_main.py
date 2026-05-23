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
from scipy.interpolate import CubicSpline
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

            self.marker_listener=self.node.create_subscription(PoseStamped,'marker_pose',self.marker_callback,10)
            self.X_Goal=None
            #TODO: Create RobotWrapper (fixed base), Call base class constructor, make publisher  
            #None
    def marker_callback(self,msg):
        p=msg.pose.position
        q=msg.pose.orientation
        self.X_Goal=pin.XYZQUATToSE3([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        
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

        # Also broadcast the hand frame
        hand_id = self._wrapper.model.getJointId("arm_right_7_joint")
        hand_pose = self._wrapper.data.oMi[hand_id]

        tf_hand = TransformStamped()
        tf_hand.header.stamp = msg.header.stamp
        tf_hand.header.frame_id = "base_link"
        tf_hand.child_frame_id = "arm_right_7_joint"

        tf_hand.transform.translation.x = hand_pose.translation[0]
        tf_hand.transform.translation.y = hand_pose.translation[1]
        tf_hand.transform.translation.z = hand_pose.translation[2]

        quat = pin.Quaternion(hand_pose.rotation)
        tf_hand.transform.rotation.x = quat.x
        tf_hand.transform.rotation.y = quat.y
        tf_hand.transform.rotation.z = quat.z
        tf_hand.transform.rotation.w = quat.w

        self.tf_broadcaster.sendTransform(tf_hand)
        
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
        self.Kp=Kp
        self.Kd=Kd
        self.robot=robot
        None
    
    def update(self, q_r, q_r_dot, q_r_ddot):
        q=self.robot.q()
        v=self.robot.v()
        model=self.robot.wrapper().model
        data=self.robot.wrapper().data

        h=pin.rnea(model,data,q,v,np.zeros(model.nv))
        M=pin.crba(model,data,q)
        M=(M+M.T) / 2.0
        tau = M @ (q_r_ddot - self.Kd @ (v-q_r_dot)- self.Kp @ (q-q_r)) + h
        return tau

        # Compute jointspace torque, return torque
        None
    
class CartesianSpaceController:
    """CartesianSpaceController
    Tracking controller in cartspace
    """
    def __init__(self, robot, joint_name, Kp, Kd):
        self.robot=robot
        self.joint_name=joint_name
        self.Kp=Kp
        self.Kd=Kd
        self.joint_id=self.robot.wrapper().model.getJointId(joint_name)
        self.damp=1e-3
        # save gains, robot ref
    def update(self, X_r, X_dot_r, X_ddot_r):

        q = self.robot.q()
        v = self.robot.v()
        model = self.robot.wrapper().model
        data = self.robot.wrapper().data

        pin.forwardKinematics(model, data, q, v, np.zeros(model.nv))
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)


        # Current pose
        X = data.oMi[self.joint_id]


        # Error in local frame — points from current toward desired
        iMd = X.actInv(X_r)
        err = pin.log(iMd).vector  # positive = need to move toward X_r

        # Raw Jacobian, no Jlog6 correction
        J = pin.computeJointJacobian(model, data, q, self.joint_id)

        # Current Cartesian velocity
        X_dot = J @ v

        # Jdot*qdot bias term
        Jdot_v = pin.getClassicalAcceleration(
            model, data, self.joint_id, pin.ReferenceFrame.LOCAL
        ).vector

        # PD law — +Kp*err because err points toward goal
        X_ddot_des = X_ddot_r - self.Kd @ (X_dot - X_dot_r) + self.Kp @ err

        # Damped pseudo-inverse
        J_pinv = J.T @ np.linalg.inv(J @ J.T + self.damp * np.eye(6))

        # Map to joint accelerations
        q_ddot_des = J_pinv @ (X_ddot_des - Jdot_v)

        # Inverse dynamics
        M = pin.crba(model, data, q)
        M = (M + M.T) / 2.0
        h = pin.rnea(model, data, q, v, np.zeros(model.nv))

        N=np.eye(model.nv) -J.T @ J_pinv.T
        return M @ q_ddot_des + h, N
            
            
  ##############################################################################
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
class JointSpline:

    def __init__(self, q_init, q_goal, duration):

        self.duration = duration

        # start/end times
        t_points = [0.0, duration]

        # shape: (2, 32)
        q_points = np.vstack([q_init, q_goal])

        # zero start/end velocity
        self.spline = CubicSpline(
            t_points,
            q_points,
            axis=0,
            bc_type=((1, np.zeros(len(q_init))),
                     (1, np.zeros(len(q_goal))))
        )

    def evaluate(self, t):
        t = np.clip(t, 0.0, self.duration)
        q = self.spline(t)
        q_dot = self.spline(t, 1)
        q_ddot = self.spline(t, 2)

        return q, q_dot, q_ddot

def main():    
    rclpy.init()

    env = Envionment()
    robot=Talos(env.simulator)
    ##set Kp and Kd
    n_joints=robot.model.nv   # 32

    Kp_diag= np.ones(n_joints) *10.0
    Kd_diag = np.ones(n_joints) * 1.0


    Kp_diag[0:12]=3 *100
    Kd_diag[0:12]=1 

    Kp_diag[12:]=1 * 20
    Kd_diag[12:]=3


    Kp=np.diag(Kp_diag)
    Kd=np.diag(Kd_diag)
    Kp_cart=np.eye(6) *5
    Kd_cart=np.eye(6) *1
    

    joint_space_controller=JointSpaceController(robot,Kp,Kd)

    q_init=robot.q().copy()
    q_home=q_init


    # new home position robot should move towards to
    a=0
    q_home[a:a+6]= np.array([0,0,0.0,0.0,0.0,0]) #left leg
    q_home[a+6:a+12]= np.array([0,0,0.0,0.0,0.0,0]) #right leg

    q_home[a+14:a+22]= np.array([0,-0.24, 0, -1, 0,0,0,0])
    q_home[a+22:a+30]= np.array([0,-0.24, 0, -1, 0,0,0,0])
    
    #q_home[a+14:a+22]= np.array([0,0.0, 0, -1, 0,0,0,0])
    #q_home[a+22:a+30]= np.array([0,0.0, 0, -1, 0,0,0,0])
    
    #todo: spline positions from q_init to q_home
    spline=JointSpline(q_init,q_home,5.0)

    cartesian_space_controller=CartesianSpaceController(robot,"arm_right_7_joint",Kp_cart,Kd_cart)

  


    # TODO: Keep looping while ros is running 
    while rclpy.ok():
        robot.update()
        rclpy.spin_once(robot.node,timeout_sec=0)
        robot.publish()


        t = env.simulator.simTime()
        dt = env.simulator.stepTime()
        
        env.update(t, dt)

        q_r,q_r_dot,q_r_ddot=spline.evaluate(t)

    
        if t<5:
            tau=joint_space_controller.update(q_r,q_r_dot,q_r_ddot)
            #robot.update()
            robot.X_Goal=robot.data().oMi[robot.wrapper().model.getJointId("arm_right_7_joint")].copy()
        else:
            X_r=robot.X_Goal          
            X_dot_r=np.zeros(6)
            X_ddot_r=np.zeros(6)
            tau_posture=joint_space_controller.update(q_r,q_r_dot,q_r_ddot)
        
            tau_cart,N=cartesian_space_controller.update(X_r,X_dot_r,X_ddot_r)
            tau=tau_cart + N @ tau_posture



        robot.setActuatedJointTorques(tau)

       
        env.simulator.debug()
        env.simulator.step()
        
if __name__ == '__main__': 
    # TODO: Todo init node
    main()
    
