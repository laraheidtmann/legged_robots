"""
Talos walking simulation — Task 2
"""

import numpy as np
import pinocchio as pin
import pybullet as pb
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

# simulator
from simulator.pybullet_wrapper import PybulletWrapper

# robot config
from . import talos_conf as conf 

# modules
from .talos          import Talos
from .footstep_planner import FootStepPlanner, Side, other_foot_id
from .foot_trajectory  import SwingFootTrajectory
from .lip_mpc          import LIPMPC, LIPInterpolator, generate_zmp_reference


################################################################################
# main
################################################################################

def main():

    ############################################################################
    # setup
    ############################################################################

    rclpy.init()
    node = rclpy.create_node('talos_walking')

    # simulator
    simulator = PybulletWrapper(sim_rate=int(1.0 / conf.dt))
    pb.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=90,
        cameraPitch=-20,
        cameraTargetPosition=[1.0, 0.0, 0.8])

    # robot (TSID + pybullet)
    robot = Talos(node, simulator)

    # initial foot poses from TSID
    T_support_w = robot.stack.get_placement_RF()   # right foot = support
    T_swing_w   = robot.stack.get_placement_LF()   # left  foot = swing


    ############################################################################
    # footstep plan
    ############################################################################

    no_steps = 20
    planner  = FootStepPlanner(conf)
    plan     = planner.planLine(T_support_w, Side.RIGHT, no_steps)

    # repeat last 2 steps so MPC horizon never runs out
    plan += [plan[-1], plan[-1]]

    # ZMP reference for the full plan
    ZMP_ref = generate_zmp_reference(plan, conf.no_mpc_samples_per_step)

    # plot footstep plan in pybullet
    planner.plot(simulator)

    ############################################################################
    # LIP MPC + interpolator
    ############################################################################

    mpc = LIPMPC(conf)

    # initial MPC state: CoM over support foot (right), zero velocity
    p_support = T_support_w.translation
    x0 = np.array([p_support[0], 0.0,
                   p_support[1], 0.0])   # [cx, vx, cy, vy]

    interpolator = LIPInterpolator(x0, conf)

    # set initial CoM task reference to support foot position
    c, c_dot, c_ddot = interpolator.comState()
    robot.stack.setComRefState(c, c_dot, c_ddot)

    # initial foot trajectory: left foot steps to plan[1] (its initial position)
    foot_traj = SwingFootTrajectory(
        T_swing_w, plan[1].poseInWorld(), conf.step_dur, height=0.05)

    ############################################################################
    # logging setup
    ############################################################################

    pre_dur = 3.0
    N_pre   = int(round(pre_dur  / conf.dt))
    N_sim   = int(round(no_steps * conf.step_dur / conf.dt))
    N_mpc   = int(round(no_steps * conf.step_dur / conf.dt_mpc))

    TIME        = np.nan * np.ones(N_sim)

    # CoM: reference (from interpolator), pinocchio
    COM_POS_REF = np.nan * np.ones((N_sim, 3))
    COM_VEL_REF = np.nan * np.ones((N_sim, 3))
    COM_ACC_REF = np.nan * np.ones((N_sim, 3))
    COM_POS_PIN = np.nan * np.ones((N_sim, 3))
    COM_VEL_PIN = np.nan * np.ones((N_sim, 3))

    # ZMP: reference, estimated
    ZMP_REF_LOG = np.nan * np.ones((N_sim, 3))
    ZMP_EST_LOG = np.nan * np.ones((N_sim, 3))

    # DCM estimated
    DCM_LOG     = np.nan * np.ones((N_sim, 3))

    # foot poses
    LF_POS_REF  = np.nan * np.ones((N_sim, 3))
    RF_POS_REF  = np.nan * np.ones((N_sim, 3))
    LF_POS_PIN  = np.nan * np.ones((N_sim, 3))
    RF_POS_PIN  = np.nan * np.ones((N_sim, 3))

    ############################################################################
    # main loop
    ############################################################################

    k              = 0       # MPC update counter
    plan_idx       = 1       # current step index in plan (start at 1: first swing)
    t_step_elapsed = 0.0     # time elapsed in current step
    t_publish      = 0.0
    u_k            = np.array([p_support[0], p_support[1]])  # initial ZMP cmd

    for i in range(-N_pre, N_sim):
        t  = simulator.simTime()
        dt = simulator.stepTime()

        ########################################################################
        # pre-walking: shift CoM to support foot and hold
        ########################################################################

        if i < 0:
            # Pre-walk: both feet in contact, shift CoM to right support foot
            robot.stack.setComRefState(c, np.zeros(3), np.zeros(3))

        if i == 0:
            # CoM is now over right foot — switch to single support
            robot.setSwingFoot(Side.LEFT)

        ########################################################################
        # MPC update every no_sim_per_mpc steps
        ########################################################################

        if i >= 0 and i % conf.no_sim_per_mpc == 0 and k < N_mpc:
            x_k       = interpolator.x
            ZMP_ref_k = ZMP_ref[k : k + conf.no_mpc_samples_per_horizon]

            terminal_idx = (no_steps * conf.no_mpc_samples_per_step) - k
            u_k = mpc.buildSolveOCP(x_k, ZMP_ref_k, terminal_idx)
            k  += 1

        ########################################################################
        # footstep transition every no_sim_per_step steps
        ########################################################################

        if i >= 0 and i > 0 and i % conf.no_sim_per_step == 0 and plan_idx + 1 < len(plan):
            next_step = plan[plan_idx]

            # next_step.side is the foot that just landed → becomes support.
            # The OTHER foot lifts off → becomes swing.
            robot.setSupportFoot(next_step.side)
            robot.setSwingFoot(other_foot_id(next_step.side))

            # swing foot travels from its current pose to the following plan entry
            T_current = robot.swingFootPose()
            T_target  = plan[plan_idx + 1].poseInWorld()
            foot_traj = SwingFootTrajectory(
                T_current, T_target, conf.step_dur, height=0.10)

            t_step_elapsed = 0.0
            plan_idx      += 1

        ########################################################################
        # every walking iteration
        ########################################################################

        if i >= 0:
            # update foot trajectory and send to swing foot
            T_foot, V_foot, A_foot = foot_traj.evaluate(t_step_elapsed)
            robot.updateSwingFootRef(T_foot, V_foot, A_foot)

            # integrate interpolator with latest MPC command
            interpolator.integrate(u_k)

            # feed CoM task with new reference
            c, c_dot, c_ddot = interpolator.comState()
            robot.stack.setComRefState(c, c_dot, c_ddot)

            t_step_elapsed += dt

        ########################################################################
        # simulator step
        ########################################################################

        simulator.step()
        simulator.debug()
        robot.update()
        rclpy.spin_once(node, timeout_sec=0)

        # publish at 30 Hz
        if t - t_publish > 1.0 / 30.0:
            t_publish = t
            robot.publish()

        ########################################################################
        # logging
        ########################################################################

        if i >= 0:
            TIME[i] = t

            c_ref, cd_ref, cdd_ref = interpolator.comState()
            COM_POS_REF[i] = c_ref
            COM_VEL_REF[i] = cd_ref
            COM_ACC_REF[i] = cdd_ref

            tsid_data = robot.stack.formulation.data()
            COM_POS_PIN[i] = robot.stack.robot.com(tsid_data)
            COM_VEL_PIN[i] = robot.stack.robot.com_vel(tsid_data)

            ZMP_REF_LOG[i] = np.array([u_k[0], u_k[1], 0.0])
            ZMP_EST_LOG[i] = robot.zmp.copy()
            DCM_LOG[i]     = robot.dcm.copy()

            LF_POS_REF[i]  = foot_traj.evaluate(t_step_elapsed)[0].translation \
                              if robot.swing_foot == Side.LEFT else \
                              robot.stack.get_placement_LF().translation
            RF_POS_REF[i]  = foot_traj.evaluate(t_step_elapsed)[0].translation \
                              if robot.swing_foot == Side.RIGHT else \
                              robot.stack.get_placement_RF().translation
            LF_POS_PIN[i]  = robot.stack.get_placement_LF().translation
            RF_POS_PIN[i]  = robot.stack.get_placement_RF().translation

    ############################################################################
    # plots
    ############################################################################

    plt.style.use('seaborn-v0_8-dark')

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    labels = ['x', 'y', 'z']
    for j in range(3):
        ax = axes[j]
        ax.plot(TIME, COM_POS_REF[:, j], label=f'CoM ref {labels[j]}')
        ax.plot(TIME, COM_POS_PIN[:, j], '--', label=f'CoM pin {labels[j]}')
        ax.set_ylabel(f'pos {labels[j]} [m]')
        ax.legend(); ax.grid(True)
    axes[2].set_xlabel('time [s]')
    fig.suptitle('CoM Position')
    plt.tight_layout()

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    for j in range(3):
        ax = axes[j]
        ax.plot(TIME, COM_VEL_REF[:, j], label=f'CoM vel ref {labels[j]}')
        ax.plot(TIME, COM_VEL_PIN[:, j], '--', label=f'CoM vel pin {labels[j]}')
        ax.set_ylabel(f'vel {labels[j]} [m/s]')
        ax.legend(); ax.grid(True)
    axes[2].set_xlabel('time [s]')
    fig.suptitle('CoM Velocity')
    plt.tight_layout()

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    for j in range(3):
        ax = axes[j]
        ax.plot(TIME, COM_ACC_REF[:, j], label=f'CoM acc ref {labels[j]}')
        ax.set_ylabel(f'acc {labels[j]} [m/s²]')
        ax.legend(); ax.grid(True)
    axes[2].set_xlabel('time [s]')
    fig.suptitle('CoM Acceleration')
    plt.tight_layout()

    fig, axes = plt.subplots(2, 1, figsize=(12, 6))
    for j, lbl in enumerate(['x', 'y']):
        ax = axes[j]
        ax.plot(TIME, ZMP_REF_LOG[:, j], '--', label=f'ZMP ref {lbl}')
        ax.plot(TIME, ZMP_EST_LOG[:, j],        label=f'ZMP est {lbl}')
        ax.set_ylabel(f'ZMP {lbl} [m]')
        ax.legend(); ax.grid(True)
    axes[1].set_xlabel('time [s]')
    fig.suptitle('Zero Moment Point')
    plt.tight_layout()

    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    for j in range(3):
        ax = axes[j]
        ax.plot(TIME, LF_POS_REF[:, j], label=f'LF ref {labels[j]}')
        ax.plot(TIME, LF_POS_PIN[:, j], '--', label=f'LF pin {labels[j]}')
        ax.plot(TIME, RF_POS_REF[:, j], label=f'RF ref {labels[j]}')
        ax.plot(TIME, RF_POS_PIN[:, j], '--', label=f'RF pin {labels[j]}')
        ax.set_ylabel(f'{labels[j]} [m]')
        ax.legend(); ax.grid(True)
    axes[2].set_xlabel('time [s]')
    fig.suptitle('Foot Positions')
    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()