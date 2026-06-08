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
    node = rclpy.create_node('tutorial_4_squatting_node')
    tsid_wrapper=TSIDWrapper(conf)
    simulator=PybulletWrapper(sim_rate=1000)
    q_home=conf.q_home
    model=tsid_wrapper.model
    urdf= conf.urdf
    ROBOT=Talos(node=node,simulator=simulator,urdf=urdf,model=model,q=q_home,useFixedBase=False)
        
    # Get current COM state (to preserve the Z height)
    com_state = tsid_wrapper.comState()
    p_com = com_state.pos().copy()  # current COM position [x, y, z]
    com_z_init=p_com[2]

    # Get current right foot position
    T_rf = tsid_wrapper.get_placement_RF()

    # Override only XY with right foot position, keep Z (height) unchanged
    p_com[0] = T_rf.translation[0]
    p_com[1] = T_rf.translation[1]
    # p_com[2] stays the same!

    # Set the new COM reference
    tsid_wrapper.setComRefState(p_com)
        
    t_publish = 0.0

    t_1=2
    t_2=4
    t_3=8
    time_elapsed_1=False
    time_elapsed_2=False
    time_elapsed_3=False
    z_LF=0.3

    # Constants:
        
    z_LF=0.3 #hight left foot

    a=0.05 #amplitude
    f=0.5 #frequency
    omega= 2* np.pi *f

    f_circle=0.1 #frequency of circle movement
    radius=0.2 #radius of circle

    #logging
    # data logging
    log_t = []
    log_com_ref = [] #COM reference
    log_com_tsid = [] #COM by TSID (what model thinks)
    log_com_pybullet = [] #COM computed by pybullet 
    log_com_vel_ref = []
    log_com_vel_tsid = []
    log_com_vel_pybullet = []
    log_com_acc_ref = []
    log_com_acc_tsid = []
    plotted=False
    try:
    
        while rclpy.ok():

            # elaped time
            t = simulator.simTime()

            simulator.step()
            simulator.debug()
            ROBOT.update()
            rclpy.spin_once(node,timeout_sec=0)
            q=ROBOT.q()
            v=ROBOT.v()
            if t>t_1 and time_elapsed_1==False:
                print("Moving foot to z: ",z_LF)
                time_elapsed_1=True
                tsid_wrapper.remove_contact_LF()
                pose=T_rf
                pose.translation[2]=z_LF
                tsid_wrapper.set_LF_pose_ref(pose)

            if t>t_2 :
                if time_elapsed_2==False:
                    print("Start squatting! ")
                    time_elapsed_2=True
                # position:
                p_com_sinusodial=p_com
                p_com_sinusodial[2] = com_z_init + a * np.sin(omega*t)
                # velocity:
                v_com=np.zeros(3)
                v_com[2]=a * np.cos(omega*t) *omega
                # acceleration:
                a_com=np.zeros(3)
                a_com[2]=-a * omega*omega * np.sin(omega*t)

                tsid_wrapper.setComRefState(p_com_sinusodial, v_com,a_com)

                
                
            if t>t_3 :
                if time_elapsed_3==False:
                    print("Start moving the hand! ")
                    time_elapsed_3=True
                # position:
                p_RH=tsid_wrapper.get_placement_RH()

                p_RH.translation[1]= -0.2 + np.sin(omega*t)*radius #y
                p_RH.translation[2]= 1.1 +  np.cos(omega*t) * radius#z
                p_RH.translation[0]=0.4  #fixed x        

                #velocity:
                v_RH=np.zeros(3)
                v_RH[1]= radius *omega * np.cos(omega*t)
                v_RH[2]= -radius * omega * np.sin(omega*t) 
                #acceleration:
                a_RH=np.zeros(3)
                a_RH[1]= -radius *omega * omega * np.sin(omega*t)
                a_RH[2]= -radius * omega *omega* np.cos(omega*t) 

                tsid_wrapper.set_RH_pose_ref(p_RH,v_RH,a_RH)




            
            # TODO: update TSID controller
            tau_sol,dv_sol= tsid_wrapper.update(q,v,t)

            # COM from TSID (what the model thinks)
            com_state = tsid_wrapper.comState()
            p_com_tsid = com_state.value().copy()
            v_com_tsid = com_state.derivative().copy()
            a_com_tsid = com_state.second_derivative().copy()

            # COM reference
            com_ref = tsid_wrapper.comReference()
            p_com_ref = com_ref.value().copy()
            v_com_ref = com_ref.derivative().copy()
            a_com_ref = com_ref.second_derivative().copy()

            # COM from PyBullet (what the simulator thinks)
            p_com_pb = ROBOT.baseWorldPosition().copy()
            v_com_pb = ROBOT.baseWorldLinearVeloctiy().copy()

            # log
            log_t.append(t)
            log_com_ref.append(p_com_ref)
            log_com_tsid.append(p_com_tsid)
            log_com_pybullet.append(p_com_pb)
            log_com_vel_ref.append(v_com_ref)
            log_com_vel_tsid.append(v_com_tsid)
            log_com_vel_pybullet.append(v_com_pb)
            log_com_acc_ref.append(a_com_ref)
            log_com_acc_tsid.append(a_com_tsid)

            if t>15:
                if DO_PLOT and plotted==False:
                    plotted=True
                    log_t_np          = np.array(log_t)
                    log_com_ref_np    = np.array(log_com_ref)
                    log_com_tsid_np   = np.array(log_com_tsid)
                    log_com_pb_np     = np.array(log_com_pybullet)
                    log_vel_ref_np    = np.array(log_com_vel_ref)
                    log_vel_tsid_np   = np.array(log_com_vel_tsid)
                    log_vel_pb_np     = np.array(log_com_vel_pybullet)
                    log_acc_ref_np    = np.array(log_com_acc_ref)
                    log_acc_tsid_np   = np.array(log_com_acc_tsid)
                
                    labels = ['X', 'Y', 'Z']
                    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
                    fig.suptitle('COM Position, Velocity, Acceleration')

                    for i in range(3):
                        # Position
                        axes[0, i].plot(log_t, log_com_ref_np[:, i],  '--', label='Ref')
                        axes[0, i].plot(log_t, log_com_tsid_np[:, i],       label='TSID')
                        axes[0, i].plot(log_t, log_com_pb_np[:, i],         label='PyBullet')
                        axes[0, i].set_title(f'COM Pos {labels[i]}')
                        axes[0, i].set_ylabel('[m]')
                        axes[0, i].legend()
                        axes[0, i].grid(True)

                        # Velocity
                        axes[1, i].plot(log_t, log_vel_ref_np[:, i],  '--', label='Ref')
                        axes[1, i].plot(log_t, log_vel_tsid_np[:, i],       label='TSID')
                        axes[1, i].plot(log_t, log_vel_pb_np[:, i],         label='PyBullet')
                        axes[1, i].set_title(f'COM Vel {labels[i]}')
                        axes[1, i].set_ylabel('[m/s]')
                        axes[1, i].legend()
                        axes[1, i].grid(True)

                        # Acceleration
                        axes[2, i].plot(log_t, log_acc_ref_np[:, i],  '--', label='Ref')
                        axes[2, i].plot(log_t, log_acc_tsid_np[:, i],       label='TSID')
                        axes[2, i].set_title(f'COM Acc {labels[i]}')
                        axes[2, i].set_ylabel('[m/s²]')
                        axes[2, i].legend()
                        axes[2, i].grid(True)

                    plt.tight_layout()
                    plt.show() 




            # TODO: command to the robot
            ROBOT.setActuatedJointTorques(tau_sol)


            # publish to ros
            if t - t_publish > 1./30.:
                t_publish = t
                T_frame_w, v_frame_w=tsid_wrapper.baseState()
                ROBOT.publish(T_frame_w)

    except KeyboardInterrupt:
        pass  
    except Exception as e:
        print("Loop crashed with:",e)
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown() 
          

if __name__ == '__main__': 
    main()
    
