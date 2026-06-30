"""Task2: Linear inverted pendulum MPC

The goal of this file is to formulate the optimal control problem (OCP)
in equation 12 but this time as a model predictive controller (MPC).

In this case we will solve the trajectory planning multiple times over 
a shorter horizon of just 2 steps (receding horizon).
Time between two MPC updates is called T_MPC.

In between MPC updates we simulate the Linear inverted pendulum at a smaller
step time T_SIM, with the lates MPC control ouput u.

Our state & control is the same as before
x = [cx, vx, cy, vy]
u = [px, py]

You will need to fill in the TODO to solve the task.
"""

import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import MathematicalProgram, Solve
from scipy.linalg import block_diag

import matplotlib.animation as animation

################################################################################
# settings
################################################################################

NO_STEPS                = 8         # total number of foot steps
STEP_TIME               = 0.8       # time needed for every step

# Robot Parameters:
# --------------
h                       = 0.80      # fixed CoM height (assuming walking on a flat terrain)
g                       = 9.81      # norm of the gravity vector
foot_length             = 0.10      # foot size in the x-direction
foot_width              = 0.06      # foot size in the y-direciton


# MPC Parameters:
# --------------
T_MPC                   = 0.1                                               # sampling time interval of the MPC
NO_MPC_SAMPLES_PER_STEP = int(round(STEP_TIME/T_MPC))                       # number of mpc updates per step

NO_STEPS_PER_HORIZON  = 2                                                   # how many steps in the horizon
T_HORIZON = NO_STEPS_PER_HORIZON*STEP_TIME                                  # duration of future horizon
NO_MPC_SAMPLES_HORIZON = int(round(NO_STEPS_PER_HORIZON*STEP_TIME/T_MPC))   # number of mpc updates per horizon

# Cost Parameters:
# ---------------
alpha       = 10**(-1)                                  # ZMP error squared cost weight (= tracking cost)
gamma       = 10**(-3)                                  # CoM velocity error squared cost weight (= smoothing cost)

# Simulation Parameters:
# --------------
T_SIM                   = 0.005                         # 200 Hz simulation time

NO_SIM_SAMPLES_PER_MPC = int(round(T_MPC/T_SIM))        # NO SIM samples between MPC updates
NO_MPC_SAMPLES = int(round(NO_STEPS*STEP_TIME/T_MPC))   # Total number of MPC samples
NO_SIM_SAMPLES = int(round(NO_STEPS*STEP_TIME/T_SIM))   # Total number of Simulator samples

################################################################################
# Helper fnc
################################################################################

def generate_foot_steps(foot_step_0, step_size_x, no_steps):
    """Write a function that generates footstep of stepsize=step_size_x in the 
    x direction starting from foot_step_0 located at (x0, y0).

    Args:
        foot_step_0 (_type_): _description_
        step_size_x (_type_): _description_
        no_steps (_type_): _description_
    """

    foot_steps = np.zeros((no_steps, 2))
    foot_steps[0] = foot_step_0
    for i in range(1, no_steps):
        x = foot_step_0[0] + (i // 2) * step_size_x
        y = foot_step_0[1] * ((-1) ** i)
        foot_steps[i] = (x, y)
    return foot_steps

def plot_foot_steps(foot_steps, XY_foot_print, ax):
    """Write a function that plots footsteps in the xy plane using the given
    footprint (length, width)
    You can use the function ax.fill() to gerneate a rectable.
    Color left and right steps differt and check if the step sequence makes sense.

    Args:
        foot_steps (_type_): _description_
    """
    lx = XY_foot_print[0] / 2
    ly = XY_foot_print[1] / 2
    for i, (x, y) in enumerate(foot_steps):
        xs = [x - lx, x + lx, x + lx, x - lx]
        ys = [y - ly, y - ly, y + ly, y + ly]
        color = 'blue' if i % 2 == 0 else 'red'
        ax.fill(xs, ys, alpha=0.5, color=color)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate a function that computes a referecne trajecotry for the zmp.
    Our goal is to keep the ZMP at the footstep center within each step

    Args:
        foot_steps (_type_): _description_
        no_samples_per_step (_type_): _description_
    """
    return np.repeat(foot_steps, no_samples_per_step, axis=0)

################################################################################
# Dynamics of the simplified walking model
################################################################################

def continious_LIP_dynamics():
    """returns the static matrices A,B of the continious LIP dynamics

    Args:
        g (_type_): gravity
        h (_type_): height

    Returns:
        np.array: A, B
    """

    A = np.array([[0,   1  ],
                  [g/h, 0  ]])
    B = np.array([[0   ],
                  [-g/h]])
    return A, B

def discrete_LIP_dynamics(dt):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics

    Args:
        dt (_type_): discretization steps

    Returns:
        _type_: _description_
    """
    import scipy.linalg
    A, B = continious_LIP_dynamics()
    Ad = scipy.linalg.expm(A * dt)
    Bd = np.linalg.inv(A) @ (Ad - np.eye(2)) @ B
    return Ad, Bd

################################################################################
# Simulation
################################################################################

class Simulator:
    """Simulates the Linear inverted pendulum continous dynamics
    Uses simple euler integration to simulate LIP at sample time dt
    """
    def __init__(self, x_inital, dt):
        self.dt = dt
        self.x = x_inital
        
        self.A, self.B = continious_LIP_dynamics()
        self.D = np.array([[0, 0], [1, 0], [0, 0], [0, 1]])
        
    def simulate(self, u, d=np.zeros(2)):
        """updates the LIP state x using based on command u
        
        Optionally: Takes a disturbance acceleration d to simulate effect
        of external pushes on the LIP.
        """

        A_full = block_diag(self.A, self.A)
        B_full = block_diag(self.B, self.B)
        x_dot = A_full @ self.x + B_full @ u + self.D @ d
        self.x = self.x + self.dt * x_dot
        return self.x

################################################################################
# MPC
################################################################################

class MPC:
    """MPC for the Linear inverted pendulum
    """
    def __init__(self, dt, T_horizon):
        self.dt = dt                                        # mpc dt
        self.T_horizon = T_horizon                          # time of horizon
        self.no_samples = int(round(T_horizon/self.dt))     # mpc samples in horizon (nodes)

        self.Ad, self.Bd = discrete_LIP_dynamics(dt)
        
        self.X_k = None                                     # state over current horizon
        self.U_k = None                                     # control over current horizon
        self.ZMP_ref_k = None                               # ZMP reference over current horizon
        
    def buildSolveOCP(self, x_k, ZMP_ref_k, terminal_idx):
        """ build the MathematicalProgram that solves the mpc problem and 
        returns the first command of U_k

        Args:
            x_k (_type_): the current state of the lip when starting the mpc
            ZMP_ref_k (_type_): the reference over the current horizon, shape=(no_samples, 2)
            terminal_idx (_type_): index of the terminal constraint within horizon (or bigger than horizon if no constraint)
            
        """
        
        # variables
        nx = 4
        nu = 2
        prog = MathematicalProgram()

        Ad_full = block_diag(self.Ad, self.Ad)
        Bd_full = block_diag(self.Bd, self.Bd)

        state = prog.NewContinuousVariables(self.no_samples, nx, 'state')
        control = prog.NewContinuousVariables(self.no_samples, nu, 'control')

        # 1. initial constraint
        for i in range(nx):
            prog.AddConstraint(state[0, i] == x_k[i])

        # 2. dynamics at every timestep
        for k in range(self.no_samples - 1):
            next_state = Ad_full @ state[k] + Bd_full @ control[k]
            for i in range(nx):
                prog.AddConstraint(state[k + 1, i] == next_state[i])

        # 3. ZMP within foot sole
        for k in range(self.no_samples):
            lb = ZMP_ref_k[k] - footprint / 2
            ub = ZMP_ref_k[k] + footprint / 2
            prog.AddConstraint(control[k, 0] >= lb[0])
            prog.AddConstraint(control[k, 0] <= ub[0])
            prog.AddConstraint(control[k, 1] >= lb[1])
            prog.AddConstraint(control[k, 1] <= ub[1])

        # 4. terminal constraint (if end of plan falls within horizon)
        if terminal_idx < self.no_samples:
            terminal_state = np.array([
                ZMP_ref_k[terminal_idx, 0], 0.0,
                ZMP_ref_k[terminal_idx, 1], 0.0
            ])
            for k in range(terminal_idx, self.no_samples):
                for i in range(nx):
                    prog.AddConstraint(state[k, i] == terminal_state[i])

        # cost: ZMP tracking + velocity smoothing
        for k in range(self.no_samples):
            zmp_err = control[k] - ZMP_ref_k[k]
            prog.AddCost(alpha * zmp_err @ zmp_err)
            prog.AddCost(gamma * (state[k, 1]**2 + state[k, 3]**2))
            
        # solve
        result = Solve(prog)
        if not result.is_success:
            print("failure")
            
        self.X_k = result.GetSolution(state)
        self.U_k = result.GetSolution(control)
        if np.isnan(self.X_k).any():
            print("failure")
        
        self.ZMP_ref_k = ZMP_ref_k
        return self.U_k[0]
    
################################################################################
# run the simulation
################################################################################

# inital state in x0 = [px0, vx0]
x_0 = np.array([0.0, 0.0])
# inital state in y0 = [py0, vy0]
y_0 = np.array([-0.09, 0.0])

# footprint
footprint = np.array([foot_length, foot_width])

# generate the footsteps
step_size = 0.2
foot_step_0 = np.array([x_0[0], y_0[0]])
foot_steps = generate_foot_steps(foot_step_0, step_size, NO_STEPS)

# repeat the last two foot steps (so the mpc horizon never exceeds the plan!)
foot_steps = np.vstack([
    foot_steps, foot_steps[-1], foot_steps[-1]])

# zmp reference trajectory
ZMP_ref = generate_zmp_reference(foot_steps, NO_MPC_SAMPLES_PER_STEP)

# generate mpc
mpc = MPC(T_MPC, T_HORIZON)

# generate the pendulum simulator
state_0 = np.concatenate([x_0, y_0])
sim = Simulator(state_0, T_SIM)

# setup some vectors for plotting stuff
TIME_VEC = np.nan*np.ones(NO_SIM_SAMPLES)
STATE_VEC = np.nan*np.ones([NO_SIM_SAMPLES, 4])
ZMP_REF_VEC = np.nan*np.ones([NO_SIM_SAMPLES, 2])
ZMP_VEC = np.nan*np.ones([NO_SIM_SAMPLES, 2])

# time to add some disturbance
t_push = 3.2

# execution loop

k = 0   # the number of mpc update
for i in range(NO_SIM_SAMPLES):
    
    # simulation time
    t = i*T_SIM
        
    if i % NO_SIM_SAMPLES_PER_MPC == 0:
        # time to update the mpc
        
        # current state
        x_k = sim.x

        # extract current horizon from full ZMP reference
        ZMP_ref_k = ZMP_ref[k : k + NO_MPC_SAMPLES_HORIZON]

        # check if we have terminal constraint
        idx_terminal_k = NO_MPC_SAMPLES - k
        u_k = mpc.buildSolveOCP(x_k, ZMP_ref_k, idx_terminal_k)

        k += 1

    # simulate a push for 0.05 sec with 1.0 m/s^2 acceleration
    x_ddot_ext = np.array([0, 0])

    # when you got everything working try adding a small disturbance
    # if i > int(t_push/T_SIM) and i < int((t_push + 0.05)/T_SIM):
    #    x_ddot_ext = np.array([0, 1.0])

    x_k = sim.simulate(u_k, x_ddot_ext)
    
    # save some stuff
    TIME_VEC[i] = t
    STATE_VEC[i] = x_k
    ZMP_VEC[i] = u_k
    ZMP_REF_VEC[i] = mpc.ZMP_ref_k[0]
    
ZMP_LB_VEC = ZMP_REF_VEC - footprint[None,:]
ZMP_UB_VEC = ZMP_REF_VEC + footprint[None,:]

A, B = continious_LIP_dynamics()
A_full = block_diag(A, A)
B_full = block_diag(B, B)
STATE_DOT_VEC = (A_full @ STATE_VEC.T + B_full @ ZMP_VEC.T).T  # (NO_SIM_SAMPLES, 4)
COM_ACC = STATE_DOT_VEC[:, [1, 3]]  # [cx_ddot, cy_ddot]

################################################################################
# plot something

fig, axes = plt.subplots(5, 1, figsize=(10, 20))

# x-axis
ax = axes[0]
ax.plot(TIME_VEC, STATE_VEC[:, 0], label='CoM x')
ax.plot(TIME_VEC, STATE_VEC[:, 1], label='CoM vx')
ax.plot(TIME_VEC, COM_ACC[:, 0],   label='CoM ax')
ax.plot(TIME_VEC, ZMP_VEC[:, 0],   label='ZMP x')
ax.plot(TIME_VEC, ZMP_REF_VEC[:, 0], '--', label='ZMP ref x')
ax.set_xlabel('time [s]')
ax.set_ylabel('x')
ax.legend()
ax.grid(True)

# CoM y position, ZMP y, ZMP ref y, bounds
ax = axes[1]
ax.plot(TIME_VEC, STATE_VEC[:, 2],    label='CoM y')
ax.plot(TIME_VEC, ZMP_VEC[:, 1],      label='ZMP y')
ax.plot(TIME_VEC, ZMP_REF_VEC[:, 1], '--', label='ZMP ref y')
ax.plot(TIME_VEC, ZMP_LB_VEC[:, 1],  ':',  label='ZMP LB y')
ax.plot(TIME_VEC, ZMP_UB_VEC[:, 1],  ':',  label='ZMP UB y')
ax.set_xlabel('time [s]')
ax.set_ylabel('y pos [m]')
ax.legend()
ax.grid(True)

# CoM y velocity
ax = axes[2]
ax.plot(TIME_VEC, STATE_VEC[:, 3], label='CoM vy')
ax.set_xlabel('time [s]')
ax.set_ylabel('y vel [m/s]')
ax.legend()
ax.grid(True)

# CoM y acceleration
ax = axes[3]
ax.plot(TIME_VEC, COM_ACC[:, 1], label='CoM ay')
ax.set_xlabel('time [s]')
ax.set_ylabel('y acc [m/s²]')
ax.legend()
ax.grid(True)

# xy-plane
ax = axes[4]
plot_foot_steps(foot_steps, footprint, ax)
ax.plot(STATE_VEC[:, 0], STATE_VEC[:, 2], color='green', label='CoM')
ax.plot(ZMP_VEC[:, 0],   ZMP_VEC[:, 1],  color='orange', linestyle='--', label='ZMP')
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.show()