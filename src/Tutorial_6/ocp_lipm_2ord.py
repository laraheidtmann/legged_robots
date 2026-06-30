"""Task2: Linear inverted pendulum Trajectory planning

The goal of this file is to formulate the optimal control problem (OCP)
in equation 12. 

In this case we will solve the trajectory planning over the entire footstep plan
(= horizon) in one go.

Our state will be the position and velocity of the pendulum in the 2d plane.
x = [cx, vx, cy, vy]
And your control the ZMP position in the 2d plane
u = [px, py]

You will need to fill in the TODO to solve the task.
"""

import numpy as np

from pydrake.all import MathematicalProgram, Solve

import matplotlib.pyplot as plt
plt.style.use('seaborn-v0_8-dark')
from scipy.linalg import block_diag

################################################################################
# settings
################################################################################

# Robot Parameters:
# --------------

h           = 0.80   # fixed CoM height (assuming walking on a flat terrain)
g           = 9.81   # norm of the gravity vector
foot_length = 0.10   # foot size in the x-direction
foot_width  = 0.06   # foot size in the y-direciton

# OCP Parameters:
# --------------
T                     = 0.1                                # fixed sampling time interval of computing the ocp in [s]
STEP_TIME             = 0.8                                # fixed time needed for every foot step [s]

NO_SAMPLES_PER_STEP   = int(round(STEP_TIME/T))            # number of ocp samples per step

NO_STEPS              = 10                                 # total number of foot steps in the plan
TOTAL_NO_SAMPLES      = NO_SAMPLES_PER_STEP*NO_STEPS       # total number of ocp samples over the complete plan (= Horizon)

# Cost Parameters:
# ---------------
alpha       = 10**(-1)                                      # ZMP error squared cost weight (= tracking cost)
gamma       = 10**(-3)                                      # CoM velocity error squared cost weight (= smoothing cost)

################################################################################
# helper function for visualization and dynamics
################################################################################

def generate_foot_steps(foot_step_0, step_size_x, no_steps):
    """Write a function that generates footstep of step size = step_size_x in the 
    x direction starting from foot_step_0 located at (x0, y0).
    
    Args:
        foot_step_0 (_type_): first footstep position (x0, y0)
        step_size_x (_type_): step size in x direction
        no_steps (_type_): number of steps to take
    """
    foot_steps = np.zeros((no_steps, 2))
    foot_steps[0] = foot_step_0
    for i in range(1, no_steps):
        x = foot_step_0[0] + (i) * step_size_x
        y = foot_step_0[1] * ((-1) ** i)
        foot_steps[i] = (x, y)



    #>>>>done: generate the foot step plan with no_steps
    #>>>>Hint: Check the pdf Fig.3 for inspiration
    return foot_steps


def plot_foot_steps(foot_steps, XY_foot_print, ax):
    """Write a function that plots footsteps in the xy plane using the given
    footprint (length, width)
    You can use the function ax.fill() to gerneate a colored rectanges.
    Color the left and right steps different and check if the step sequence makes sense.

    Args:
        foot_steps (_type_): the foot step plan
        XY_foot_print (_type_): the dimensions of the foot (x,y)
        ax (_type_): the axis to plot on
    """
    lx = XY_foot_print[0] / 2  # half foot length
    ly = XY_foot_print[1] / 2  # half foot width

    for i, (x, y) in enumerate(foot_steps):
        xs = [x - lx, x + lx, x + lx, x - lx]
        ys = [y - ly, y - ly, y + ly, y + ly]

        color = 'blue' if i % 2 == 0 else 'red'
        ax.fill(xs, ys, alpha=0.5, color=color)

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')
    #>>>>done: Plot the the footsteps into ax 

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate a function that computes a referecne trajecotry for the ZMP
    (We need this for the tracking cost in the cost function of eq. 12)
    Remember: Our goal is to keep the ZMP at the footstep center within each step.
    So for the # of samples a step is active the zmp_ref should be at that step.
    
    Returns a vector of size (TOTAL_NO_SAMPLES, 2)

    Args:
        foot_steps (_type_): the foot step plan
        no_samples_per_step (_type_): number of sampes per step
    """
    #>>>>done: Generate the ZMP reference based on given foot_steps
    zmp_ref = None
    zmp_ref= np.repeat(foot_steps, no_samples_per_step, axis=0)
    return zmp_ref

################################################################################
# Dynamics of the simplified walking model
################################################################################

def continious_LIP_dynamics(g, h):
    """returns the matrices A,B of the continious LIP dynamics

    Args:
        g (_type_): gravity
        h (_type_): fixed height

    Returns:
        np.array: A, B
    """
    #>>>>done: Generate A, B for the continous linear inverted pendulum
    #>>>>Hint: Look at Eq. 4 and rewrite as a system first order diff. eq.
    A=None; B=None
    A=np.array([[0, 1],
        [g/h,0]])
    B= np.array([[0],[-g/h]])
    return A, B

def discrete_LIP_dynamics(delta_t, g, h):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics

    Args:
        delta_t (_type_): discretization steps
        g (_type_): gravity
        h (_type_): height

    Returns:
        _type_: _description_
    """
    #>>>>done: Generate Ad, Bd for the discretized linear inverted pendulum
    import scipy.linalg    
    Ad=None; Bd=None
    
    A, B = continious_LIP_dynamics(g, h)
    
    Ad = scipy.linalg.expm(A * delta_t)
    Bd = np.linalg.inv(A) @ (Ad - np.eye(2)) @ B
    
    return Ad, Bd

################################################################################
# setup the plan references and system matrices
################################################################################

# inital state in x0 = [px0, vx0]
x_0 = np.array([0.0, 0.0])
# inital state in y0 = [py0, vy0]
y_0 = np.array([-0.09, 0.0])

# footprint
footprint = np.array([foot_length, foot_width])

# generate the footsteps
step_size = 0.2
#>>>>done: 1. generate the foot step plan using generate_foot_steps
foot_step_0=np.array([x_0[0],y_0[0]])
foot_steps=generate_foot_steps(foot_step_0,step_size,NO_STEPS )

# zmp reference trajecotry
#>>>>done: 2. generate the ZMP reference using generate_zmp_reference
zmp_ref=generate_zmp_reference(foot_steps,NO_SAMPLES_PER_STEP)
#>>>>Note: At this point you can already start plotting things to see if they
# really make sense!
fig,ax=plt.subplots()
plot_foot_steps(foot_steps,footprint,ax)
plt.show()

# discrete LIP dynamics
delta_t=0.1
Ad,Bd=discrete_LIP_dynamics(delta_t,g,h)

#>>>>done: get the static dynamic matrix Ad, Bd

# continous LIP dynamics
#>>>>done: get the static dynamic matrix A, B
A,B=continious_LIP_dynamics(g,h)

################################################################################
# problem definition
################################################################################

# Define an instance of MathematicalProgram 
prog = MathematicalProgram() 

################################################################################
# variables
nx = 4 #>>>>done: State dimension = ?
nu = 2 #>>>>done: control dimension = ?

state = prog.NewContinuousVariables(TOTAL_NO_SAMPLES, nx, 'state')
control = prog.NewContinuousVariables(TOTAL_NO_SAMPLES, nu, 'control')

# intial state
state_inital = np.concatenate([x_0,y_0]) #>>>>done: inital state if based on first footstep (+ zero velo)

# terminal state
state_terminal = np.array([foot_steps[-1,0],0.0,foot_steps[-1,1],0.0]) #>>>>done: terminal state if based on last footstep (+ zero velo)

################################################################################
# constraints

# 1. intial constraint
#>>>>TODO: Add inital state constrain, Hint: prog.AddConstraint
for i in range(nx):
    prog.AddConstraint(state[0,i] == state_inital[i])


# 2. terminal constraint
#>>>>TODO: Add terminal state constrain, Hint: prog.AddConstraint
for i in range(nx):
    prog.AddConstraint(state[-1,i]==state_terminal[i])
# 3. at each step: respect the LIP descretized dynamics
#>>>>TODO: Enforce the dynamics at every time step
Ad_full=block_diag(Ad,Ad)
Bd_full=block_diag(Bd,Bd)
for k in range(TOTAL_NO_SAMPLES -1):
    next_state= Ad_full @ state[k] + Bd_full @ control[k]
    for i in range(nx):
        prog.AddConstraint(state[k+1,i] == next_state[i])

# 4. at each step: keep the ZMP within the foot sole (use the footprint and planned step position)
#>>>>TODO: Add ZMP upper and lower bound to keep the control (ZMP) within each footprints
#Hint: first compute upper and lower bound based on zmp_ref then add constraints.
#Hint: Add constraints at every time step
for k in range(TOTAL_NO_SAMPLES-1):
    lb= zmp_ref[k] - footprint/2
    ub= zmp_ref[k] + footprint/2
    prog.AddBoundingBoxConstraint(lb,ub,control[k])
    

################################################################################
# stepwise cost, note that the cost function is scalar!

# setup our cost: minimize zmp error (tracking), minimize CoM velocity (smoothing)
#>>>>TODO: add the cost at each timestep, hint: prog.AddCost
for k in range(TOTAL_NO_SAMPLES):
    zmp_err=control[k]-zmp_ref[k]
    prog.AddCost(alpha * zmp_err @ zmp_err)
    prog.AddCost(gamma * (state[k,1]**2 + state[k,3]**2))
################################################################################
# solve

result = Solve(prog)
if not result.is_success:
    print("failure")
print("solved")

# extract the solution
#>>>>TODO: extract your variables from the result object
t = T*np.arange(0, TOTAL_NO_SAMPLES)
state_sol = result.GetSolution(state)
control_sol = result.GetSolution(control)

# compute the acceleration
#>>>>TODO: compute the acceleration of the COM
A_full=block_diag(A,A)
B_full=block_diag(B,B)
state_dot = (A_full @ state_sol.T + B_full @ control_sol.T).T  # (TOTAL_NO_SAMPLES, 4)

com_acc = state_dot[:, [1, 3]]  # [cx_ddot, cy_ddot]
################################################################################
# plot something
fig, axes = plt.subplots(3, 1, figsize=(10, 12))

# x-axis
ax = axes[0]
ax.plot(t, state_sol[:, 0], label='CoM x')
ax.plot(t, state_sol[:, 1], label='CoM vx')
ax.plot(t, com_acc[:, 0],   label='CoM ax')
ax.plot(t, control_sol[:, 0], label='ZMP x')
ax.plot(t, zmp_ref[:, 0],   label='ZMP ref x', linestyle='--')
ax.set_xlabel('time [s]')
ax.set_ylabel('x [m]')
ax.legend()
ax.grid(True)

# y-axis
ax = axes[1]
ax.plot(t, state_sol[:, 2], label='CoM y')
ax.plot(t, state_sol[:, 3], label='CoM vy')
ax.plot(t, com_acc[:, 1],   label='CoM ay')
ax.plot(t, control_sol[:, 1], label='ZMP y')
ax.plot(t, zmp_ref[:, 1],   label='ZMP ref y', linestyle='--')
ax.set_xlabel('time [s]')
ax.set_ylabel('y [m]')
ax.legend()
ax.grid(True)

# xy-plane
ax = axes[2]
plot_foot_steps(foot_steps, footprint, ax)
ax.plot(state_sol[:, 0], state_sol[:, 2], label='CoM', color='green')
ax.plot(control_sol[:, 0], control_sol[:, 1], label='ZMP', color='orange', linestyle='--')
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.show()
#>>>>TODO: plot everything in x-axis
#>>>>TODO: plot everything in y-axis
#>>>>TODO: plot everything in xy-plane