import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt


class SwingFootTrajectory:
    """SwingFootTrajectory
    Interpolate Foot trajectory between SE3 T0 and T1 using a 5th degree
    polynomial (smoothstep) for x/y and a smoothstep + bump for z.

    Guarantees:
        - zero velocity and acceleration at t=0 and t=duration
        - z(duration/2) = height (peak at midpoint)
        - constant foot orientation
    """
    def __init__(self, T0, T1, duration, height=0.05):
        self._height = height
        self._t_elapsed = 0.0
        self._duration = duration
        self.reset(T0, T1)

    def reset(self, T0, T1):
        """reset back to zero, update poses"""
        self._t_elapsed = 0.0
        self._p0 = T0.translation.copy()
        self._p1 = T1.translation.copy()
        self._R  = T0.rotation.copy()   # constant orientation

    def isDone(self):
        return self._t_elapsed >= self._duration

    def _smoothstep(self, tau):
        """5th-degree smoothstep in normalised time tau in [0,1].
        f(0)=0, f(1)=1, f'(0)=f'(1)=f''(0)=f''(1)=0
        Returns f, df/dtau, d2f/dtau2
        """
        f   = 10*tau**3 - 15*tau**4  + 6*tau**5
        df  = 30*tau**2 - 60*tau**3  + 30*tau**4
        ddf = 60*tau    - 180*tau**2  + 120*tau**3
        return f, df, ddf

    def _bump(self, tau):
        """Bump function b(tau) = tau^3*(1-tau)^3.
        b(0)=b(1)=0, b'(0)=b'(1)=0, b''(0)=b''(1)=0, b(0.5)=1/64
        Returns b, db/dtau, d2b/dtau2
        """
        b   = tau**3 * (1 - tau)**3
        db  = 3*tau**2*(1 - tau)**2*(1 - 2*tau)
        ddb = (6*tau*(1 - tau)**2*(1 - 2*tau)
               - 6*tau**2*(1 - tau)*(1 - 2*tau)
               - 6*tau**2*(1 - tau)**2)
        return b, db, ddb

    def evaluate(self, t):
        """Evaluate trajectory at elapsed time t.

        Args:
            t (float): elapsed time since start of step [s]

        Returns:
            pose (pin.SE3): foot pose
            vel  (np.array 3): linear velocity [m/s]
            acc  (np.array 3): linear acceleration [m/s^2]
        """
        self._t_elapsed = t
        T   = self._duration
        tau = np.clip(t / T, 0.0, 1.0)

        p0, p1 = self._p0, self._p1
        f, df, ddf = self._smoothstep(tau)

        # --- position ---
        pos      = p0 + (p1 - p0) * f
        # lift z above the straight-line path using the bump
        # bump peaks at tau=0.5 with value 1/64, so scale by 64*h_lift
        h_lift   = self._height - (p0[2] + p1[2]) / 2.0
        b, db, ddb = self._bump(tau)
        pos[2]  += 64.0 * h_lift * b

        # --- velocity  (chain rule: d/dt = d/dtau * 1/T) ---
        vel      = (p1 - p0) * df / T
        vel[2]  += 64.0 * h_lift * db / T

        # --- acceleration (chain rule: d2/dt2 = d2/dtau2 * 1/T^2) ---
        acc      = (p1 - p0) * ddf / T**2
        acc[2]  += 64.0 * h_lift * ddb / T**2

        pose = pin.SE3(self._R.copy(), pos)
        return pose, vel, acc


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    T0 = pin.SE3(np.eye(3), np.array([0.0,  0.096, 0.0]))
    T1 = pin.SE3(np.eye(3), np.array([0.25, 0.096, 0.0]))

    duration = 0.8
    height   = 0.05
    traj = SwingFootTrajectory(T0, T1, duration, height)

    dt    = 0.001
    times = np.arange(0.0, duration + dt, dt)

    pos_arr = np.zeros((len(times), 3))
    vel_arr = np.zeros((len(times), 3))
    acc_arr = np.zeros((len(times), 3))

    for i, t in enumerate(times):
        pose, vel, acc = traj.evaluate(t)
        pos_arr[i] = pose.translation
        vel_arr[i] = vel
        acc_arr[i] = acc

    # verify boundary conditions
    print("=== Boundary check ===")
    print(f"pos(0)   = {pos_arr[0]}   (should be {T0.translation})")
    print(f"pos(T)   = {pos_arr[-1]}  (should be {T1.translation})")
    print(f"pos(T/2) z = {pos_arr[len(times)//2, 2]:.4f}  (should be {height})")
    print(f"vel(0)   = {vel_arr[0]}   (should be [0,0,0])")
    print(f"vel(T)   = {vel_arr[-1]}  (should be [0,0,0])")
    print(f"acc(0)   = {acc_arr[0]}   (should be [0,0,0])")
    print(f"acc(T)   = {acc_arr[-1]}  (should be [0,0,0])")

    fig, axes = plt.subplots(3, 1, figsize=(10, 9))

    ax = axes[0]
    ax.plot(times, pos_arr[:, 0], label='x')
    ax.plot(times, pos_arr[:, 1], label='y')
    ax.plot(times, pos_arr[:, 2], label='z')
    ax.axhline(height, color='k', linestyle='--', label=f'h_step={height}')
    ax.set_ylabel('position [m]')
    ax.legend(); ax.grid(True)

    ax = axes[1]
    ax.plot(times, vel_arr[:, 0], label='vx')
    ax.plot(times, vel_arr[:, 1], label='vy')
    ax.plot(times, vel_arr[:, 2], label='vz')
    ax.set_ylabel('velocity [m/s]')
    ax.legend(); ax.grid(True)

    ax = axes[2]
    ax.plot(times, acc_arr[:, 0], label='ax')
    ax.plot(times, acc_arr[:, 1], label='ay')
    ax.plot(times, acc_arr[:, 2], label='az')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('acceleration [m/s²]')
    ax.legend(); ax.grid(True)

    plt.suptitle('Swing Foot Trajectory')
    plt.tight_layout()
    plt.show()