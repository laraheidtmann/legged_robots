import pybullet as pb
import numpy as np
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot
import pinocchio as pin

# For REEM-C robot
#urdf = "src/reemc_description/robots/reemc.urdf"
#path_meshes = "src/reemc_description/meshes/../.."

# For Talos robot
urdf = "src/talos_description/robots/talos_reduced.urdf"
path_meshes = "src/talos_description/meshes/../.."

'''
Talos
0, 1, 2, 3, 4, 5, 			    # left leg
6, 7, 8, 9, 10, 11, 			# right leg
12, 13,                         # torso
14, 15, 16, 17, 18, 19, 20, 21  # left arm
22, 23, 24, 25, 26, 27, 28, 29  # right arm
30, 31                          # head

REEMC
0, 1, 2, 3, 4, 5, 			    # left leg
6, 7, 8, 9, 10, 11, 			# right leg
12, 13,                         # torso
14, 15, 16, 17, 18, 19, 20,     # left arm
21, 22, 23, 24, 25, 26, 27,     # right arm
28, 29                          # head
'''

# Initial condition for the simulator an model
z_init = 1.15
q_actuated_home = np.zeros(32)
q_actuated_home[:6] = np.array([0, 0, 0, 0, 0, 0])
q_actuated_home[6:12] = np.array([0, 0, 0, 0, 0, 0])
q_actuated_home[14:22] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])
q_actuated_home[22:30] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])

# Initialization position including floating base
q_home = np.hstack([np.array([0, 0, z_init, 0, 0, 0, 1]), q_actuated_home])


# setup the task stack
modelWrap = pin.RobotWrapper.BuildFromURDF(urdf,                        # Model description
                                           path_meshes,                 # Model geometry descriptors 
                                           pin.JointModelFreeFlyer(),   # Floating base model. Use "None" if fixed
                                           True,                        # Printout model details
                                           None)                        # Load meshes different from the descripor
# Get model from wrapper
model = modelWrap.model


# setup the simulator
simulator = PybulletWrapper(sim_rate=1000)

#Create Pybullet-Pinocchio map
robot = Robot(simulator,            # The Pybullet wrapper
              urdf,                 # Robot descriptor
              model,                # Pinocchio model
              [0, 0, z_init],       # Floating base initial position
              [0,0,0,1],            # Floating base initial orientation [x,y,z,w]
              q=q_home,             # Initial state
              useFixedBase=False,   # Fixed base or not
              verbose=True)         # Printout details

#Needed for compatibility
simulator.addLinkDebugFrame(-1,-1)

#########TODO build pinocchio data structure of the model
data = robot._model.createData()
M= pin.crba(model,data,q_home)
print("-------------   complete intertia matrix: -------------")
print(M)
print("-------------   non linear effects: -------------")

v=np.zeros(model.nv) #joint velocity
nle=pin.nonLinearEffects(model,data,q_home,v)
print(nle)
#########

# Setup pybullet camera
pb.resetDebugVisualizerCamera(
    cameraDistance=1.2,
    cameraYaw=90,
    cameraPitch=-20,
    cameraTargetPosition=[0.0, 0.0, 0.8])

# Joint command vector
tau = q_actuated_home*0

#####TODO: implement joint space PD Controller
n_joints=model.nv -6  # 32
qd=np.zeros(n_joints)

Kp_diag= np.ones(n_joints) *10.0
Kd_diag = np.ones(n_joints) * 1.0


Kp_diag[0:12]=3 *300
Kd_diag[0:12]=1 

Kp_diag[12:]=1 * 300
Kd_diag[12:]=1 


Kp=np.diag(Kp_diag)
Kd=np.diag(Kd_diag)

#####

def spline_joint_positions(q_ini,q_home,t,T):
    if t>=T:
        return q_home.copy()
    #normalize time
    s=t/T
    alpha=3.0 * s**2 -2.0 * s**3

    return pin.interpolate(model,q_ini,q_home,alpha)

T_spline=2.0
t_sim=0.0
dt=1.0/1000
q_ini=robot.q().copy()

#new q_home:
#TODO: new home position:
q_home[:6]= np.array([0,0,-0.44,0.9,-0.45,0]) #left leg
q_home[6:12]= np.array([0,0,-0.44,0.9,-0.45,0]) #right leg

q_home[14:21]= np.array([0,-0.24, 0, -1, 0,0,0])
q_home[21:28]= np.array([0,-0.24, 0, -1, 0,0,0])





done = False
while not done:
    # update the simulator and the robot
    simulator.step()
    simulator.debug()
    robot.update()
    
    q_full=robot.q()
    v_full=robot.v()

    q=q_full[7:]
    v=v_full[6:]

    q_desired_full=spline_joint_positions(q_ini,q_home,t_sim,T_spline)
    t_sim+=dt
    qd=q_desired_full[7:]
    print(np.round(qd,2))


    tau=Kp @(qd-q) - Kd @ v
    
    # command to the robot
    robot.setActuatedJointTorques(tau)