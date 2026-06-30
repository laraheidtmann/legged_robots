import numpy as np
import pinocchio as pin
from enum import Enum
import pybullet as pb


class Side(Enum):
    """Side — which foot"""
    LEFT  = 0
    RIGHT = 1


def other_foot_id(id):
    return Side.RIGHT if id == Side.LEFT else Side.LEFT


class FootStep:
    """Holds all information describing a single footstep."""

    def __init__(self, pose, footprint, side=Side.LEFT):
        """
        Args:
            pose (pin.SE3): pose of the footstep in world frame
            footprint (np.array): 3×4 matrix of foot corner vertices (foot frame)
            side (Side): which foot
        """
        self.pose      = pose
        self.footprint = footprint
        self.side      = side

    def poseInWorld(self):
        return self.pose

    def plot(self, simulation):
        """Visualise footstep in PyBullet."""
        pos = self.pose.translation
        R   = self.pose.rotation
        # quaternion from rotation matrix (x,y,z,w)
        Q = pin.Quaternion(R)
        Q_pb = [Q.x, Q.y, Q.z, Q.w]

        # foot rectangle dimensions from footprint vertices
        length = self.footprint[0].max() - self.footprint[0].min()  # lfxp + lfxn
        width  = self.footprint[1].max() - self.footprint[1].min()  # lfyp + lfyn

        # centre of the footprint in foot frame → world frame
        fp_centre_local = np.array([
            (self.footprint[0].max() + self.footprint[0].min()) / 2.0,
            0.0,
            0.0
        ])
        centre_world = pos + R @ fp_centre_local

        # draw rectangle
        simulation.addGlobalDebugRectancle(
            centre_world, Q_pb,
            length=length, width=width,
            color=[0, 0, 1] if self.side == Side.LEFT else [1, 0, 0]
        )

        # label LEFT / RIGHT
        label = "LEFT" if self.side == Side.LEFT else "RIGHT"
        pb.addUserDebugText(label, pos + np.array([0, 0, 0.05]),
                            textColorRGB=[0, 0, 0], textSize=1.0)

        # sphere at step target
        simulation.addSphereMarker(
            position=pos.tolist(),
            color=[0, 0, 1, 1] if self.side == Side.LEFT else [1, 0, 0, 1]
        )


class FootStepPlanner:
    """Creates footstep plans (list of FootStep objects)."""

    def __init__(self, conf):
        self.conf  = conf
        self.steps = []

    def planLine(self, T_0_w, side, no_steps):
        """Plan a sequence of steps in a straight line.

        The second step is parallel to the first (robot starts on both feet).
        The last step is parallel to the second-to-last (robot stops on both feet).
        X advances every two steps; y alternates between the two feet.

        Args:
            T_0_w  (pin.SE3): pose of the first step
            side   (Side):    which foot takes the first step
            no_steps (int):   total number of steps

        Returns:
            list[FootStep]
        """
        dx = self.conf.step_size_x
        dy = 2 * self.conf.step_size_y

        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn

        # 4-corner footprint in foot frame (3×4)
        footprint = np.array([
            [ lfxp,  lfxp, -lfxn, -lfxn],
            [ lfyp, -lfyn, -lfyn,  lfyp],
            [ 0.,    0.,    0.,    0.  ]
        ])

        # y offset to reach the other foot (relative to T_0_w)
        y_other = -dy if side == Side.LEFT else +dy

        steps = []
        for k in range(no_steps):
            T = pin.SE3(T_0_w.rotation.copy(), T_0_w.translation.copy())

            # steps 0 and 1: same x (start with both feet parallel)
            # steps 2..no_steps-2: advance by dx each step (versetzt/staggered)
            # last step: same x as second-to-last (end with both feet parallel)
            if k <= 1:
                x_offset = 0
            elif k == no_steps - 1:
                x_offset = (no_steps - 3) * dx   # same as step no_steps-2
            else:
                x_offset = (k - 1) * dx

            T.translation[0] += x_offset

            # y: even steps stay at T_0_w y, odd steps are the other foot
            if k % 2 == 1:
                T.translation[1] += y_other

            s = side if k % 2 == 0 else other_foot_id(side)
            steps.append(FootStep(T, footprint, s))

        self.steps = steps
        return steps

    def plot(self, simulation):
        for step in self.steps:
            step.plot(simulation)


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    """Test footstep planner with a matplotlib plot (no pybullet needed)."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.patches import FancyArrowPatch

    # minimal config mock
    class Conf:
        step_size_x = 0.25
        step_size_y = 0.096
        lfxp = 0.12
        lfxn = 0.08
        lfyp = 0.065
        lfyn = 0.065

    conf = Conf()

    # starting pose: right foot at (0, -step_size_y)
    T_start = pin.SE3(np.eye(3), np.array([0.0, -conf.step_size_y, 0.0]))
    planner = FootStepPlanner(conf)
    steps   = planner.planLine(T_start, Side.RIGHT, no_steps=10)

    fig, ax = plt.subplots(figsize=(12, 4))

    lx = (conf.lfxp + conf.lfxn) / 2
    ly = (conf.lfyp + conf.lfyn) / 2

    for i, step in enumerate(steps):
        x, y = step.pose.translation[0], step.pose.translation[1]
        color = 'royalblue' if step.side == Side.LEFT else 'tomato'
        rect = mpatches.FancyBboxPatch(
            (x - lx, y - ly), 2*lx, 2*ly,
            boxstyle="round,pad=0.005",
            linewidth=1.5, edgecolor='black',
            facecolor=color, alpha=0.6
        )
        ax.add_patch(rect)
        label = 'L' if step.side == Side.LEFT else 'R'
        ax.text(x, y, f'{label}{i}', ha='center', va='center', fontsize=8)

    # connect step sequence with arrows
    xs = [s.pose.translation[0] for s in steps]
    ys = [s.pose.translation[1] for s in steps]
    ax.plot(xs, ys, 'k--', linewidth=0.8, alpha=0.4)

    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'Footstep plan ({len(steps)} steps)')
    left_patch  = mpatches.Patch(color='royalblue', alpha=0.6, label='Left')
    right_patch = mpatches.Patch(color='tomato',    alpha=0.6, label='Right')
    ax.legend(handles=[left_patch, right_patch])
    ax.grid(True)
    plt.tight_layout()
    plt.show()

    # print summary
    print(f"{'#':>3}  {'side':>6}  {'x':>6}  {'y':>7}")
    for i, s in enumerate(steps):
        print(f"{i:>3}  {s.side.name:>6}  {s.pose.translation[0]:>6.3f}  {s.pose.translation[1]:>7.3f}")