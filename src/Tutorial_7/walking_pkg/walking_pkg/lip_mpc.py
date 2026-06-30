import numpy as np
import scipy.linalg
from scipy.linalg import block_diag

from pydrake.all import MathematicalProgram, Solve

################################################################################
# Dynamics
################################################################################

def continious_LIP_dynamics(g, h):
    """Continuous LIP dynamics: ẋ = A x + B u
    state x = [c, ċ],  control u = p (ZMP)
    """
    A = np.array([[0.,   1. ],
                  [g/h,  0. ]])
    B = np.array([[0.  ],
                  [-g/h]])
    return A, B


def discrete_LIP_dynamics(g, h, dt):
    """Discretised LIP dynamics via matrix exponential (ZOH)."""
    A, B = continious_LIP_dynamics(g, h)
    Ad = scipy.linalg.expm(A * dt)
    Bd = np.linalg.inv(A) @ (Ad - np.eye(2)) @ B
    return Ad, Bd


################################################################################
# ZMP reference from footstep plan
################################################################################

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """Build a ZMP reference trajectory from a list of FootStep objects.

    For each step the ZMP reference stays at the foot centre for
    no_samples_per_step samples.

    Args:
        foot_steps (list[FootStep]): footstep plan
        no_samples_per_step (int):  MPC samples per step

    Returns:
        np.array (N, 2): ZMP reference in x and y
    """
    zmp_ref = np.zeros((len(foot_steps) * no_samples_per_step, 2))
    for i, step in enumerate(foot_steps):
        pos = step.poseInWorld().translation
        zmp_ref[i * no_samples_per_step:(i + 1) * no_samples_per_step, 0] = pos[0]
        zmp_ref[i * no_samples_per_step:(i + 1) * no_samples_per_step, 1] = pos[1]
    return zmp_ref


################################################################################
# LIP Interpolator — runs at full simulator frequency
################################################################################

class LIPInterpolator:
    """Integrates the LIP continuous dynamics at the simulator timestep dt
    to provide high-frequency CoM references between MPC updates.
    """

    def __init__(self, x_initial, conf):
        """
        Args:
            x_initial (np.array 4): initial MPC state [cx, vx, cy, vy]
            conf: robot/walking config (uses conf.dt, conf.g, conf.h)
        """
        self.conf = conf
        self.dt   = conf.dt
        self.x    = x_initial.copy()   # [cx, vx, cy, vy]
        self.u    = np.zeros(2)         # latest ZMP command [px, py]

        A, B = continious_LIP_dynamics(conf.g, conf.h)
        self.A_full = block_diag(A, A)  # 4×4
        self.B_full = block_diag(B, B)  # 4×2

    def setState(self, x):
        self.x = x.copy()

    def integrate(self, u):
        """Euler-integrate one step with control u = [px, py].

        Args:
            u (np.array 2): ZMP command
        Returns:
            np.array 4: updated state
        """
        self.u   = u
        x_dot    = self.A_full @ self.x + self.B_full @ u
        self.x   = self.x + self.dt * x_dot
        return self.x

    def comState(self):
        """Return CoM position, velocity, acceleration in 3D (z = conf.h).

        Returns:
            c      (np.array 3): position
            c_dot  (np.array 3): velocity
            c_ddot (np.array 3): acceleration
        """
        x_dot  = self.A_full @ self.x + self.B_full @ self.u
        c      = np.array([self.x[0], self.x[2], self.conf.h])
        c_dot  = np.array([self.x[1], self.x[3], 0.0])
        c_ddot = np.array([x_dot[1],  x_dot[3],  0.0])
        return c, c_dot, c_ddot

    def dcm(self):
        """Divergent Component of Motion: ξ = c + ċ / ω₀,  ω₀ = √(g/h)."""
        omega = np.sqrt(self.conf.g / self.conf.h)
        c, c_dot, _ = self.comState()
        return c + c_dot / omega

    def zmp(self):
        """Return current ZMP command as 3D point (z = 0)."""
        return np.array([self.u[0], self.u[1], 0.0])


################################################################################
# LIP MPC — runs at MPC frequency
################################################################################

class LIPMPC:
    """Receding-horizon MPC for the Linear Inverted Pendulum."""

    def __init__(self, conf):
        self.conf       = conf
        self.dt         = conf.dt_mpc
        self.no_samples = conf.no_mpc_samples_per_horizon

        Ad, Bd = discrete_LIP_dynamics(conf.g, conf.h, conf.dt_mpc)
        self.Ad_full = block_diag(Ad, Ad)   # 4×4
        self.Bd_full = block_diag(Bd, Bd)   # 4×2

        self.footprint = np.array([conf.lfxp + conf.lfxn,
                                   conf.lfyp + conf.lfyn])

        # cost weights
        self.alpha = 1e-1   # ZMP tracking
        self.gamma = 1e-3   # velocity smoothing

        # solution storage
        self.X_k       = None
        self.U_k       = None
        self.ZMP_ref_k = None

    def buildSolveOCP(self, x_k, ZMP_ref_k, terminal_idx):
        """Build and solve the receding-horizon OCP.

        Args:
            x_k         (np.array 4): current LIP state [cx,vx,cy,vy]
            ZMP_ref_k   (np.array no_samples×2): ZMP reference over horizon
            terminal_idx (int): horizon index at which to apply terminal
                                constraint (set > no_samples to skip)
        Returns:
            np.array 2: first ZMP command [px, py]
        """
        nx = 4
        nu = 2
        N  = self.no_samples

        prog    = MathematicalProgram()
        state   = prog.NewContinuousVariables(N, nx, 'state')
        control = prog.NewContinuousVariables(N, nu, 'control')

        # 1. initial constraint
        for i in range(nx):
            prog.AddConstraint(state[0, i] == x_k[i])

        # 2. dynamics
        for k in range(N - 1):
            next_s = self.Ad_full @ state[k] + self.Bd_full @ control[k]
            for i in range(nx):
                prog.AddConstraint(state[k + 1, i] == next_s[i])

        # 3. ZMP within foot sole
        for k in range(N):
            lb = ZMP_ref_k[k] - self.footprint / 2
            ub = ZMP_ref_k[k] + self.footprint / 2
            prog.AddConstraint(control[k, 0] >= lb[0])
            prog.AddConstraint(control[k, 0] <= ub[0])
            prog.AddConstraint(control[k, 1] >= lb[1])
            prog.AddConstraint(control[k, 1] <= ub[1])

        # 4. terminal constraint (if end of plan is within horizon)
        if terminal_idx < N:
            term = np.array([ZMP_ref_k[terminal_idx, 0], 0.0,
                             ZMP_ref_k[terminal_idx, 1], 0.0])
            for k in range(terminal_idx, N):
                for i in range(nx):
                    prog.AddConstraint(state[k, i] == term[i])

        # 5. cost
        for k in range(N):
            zmp_err = control[k] - ZMP_ref_k[k]
            prog.AddCost(self.alpha * zmp_err @ zmp_err)
            prog.AddCost(self.gamma * (state[k, 1]**2 + state[k, 3]**2))

        result = Solve(prog)
        if not result.is_success():
            print("[LIPMPC] warning: solver did not converge")

        self.X_k       = result.GetSolution(state)
        self.U_k       = result.GetSolution(control)
        self.ZMP_ref_k = ZMP_ref_k

        if np.isnan(self.U_k).any():
            print("[LIPMPC] warning: NaN in solution, keeping previous command")
            self.U_k = np.zeros((N, nu))

        return self.U_k[0]