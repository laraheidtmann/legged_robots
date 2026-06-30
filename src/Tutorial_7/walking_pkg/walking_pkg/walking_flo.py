"""
talos walking simulation
"""

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

# simulator
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot config
from . import talos_conf as conf 

# modules
from .talos_flo          import Talos
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
    # this is also at the very end of this file, but starting scripts with 
    # ros2 run skips the _if name == main_ stuff 
    if not rclpy.ok():
        rclpy.init()

    # setup the simulator
    frequency = 300
    conf.dt  = 1.0/frequency
    conf.no_sim_per_mpc = int(round(conf.dt_mpc / conf.dt))
    conf.no_sim_per_step = int(round(conf.step_dur / conf.dt))
    simulator = PybulletWrapper(sim_rate=frequency, num_sub_steps=8, render=True)

    # setup the robot
    robot = Talos(simulator)

    # inital footsteps
    T_swing_w = robot.swingFootPose()
    T_support_w = robot.supportFootPose()
    
    # setup the plan with 20 steps
    # seed the plan with the current SUPPORT side and pose: plan[0] is the
    # foot already in contact (stepping starts at plan_idx=1), so plan[0].side
    # must match robot.support_foot or the first swing lifts the loaded foot
    no_steps = 20
    planner = FootStepPlanner(conf)
    plan = planner.planLine(T_support_w, robot.support_foot, no_steps=no_steps)
    # repeat the final step twice so the mpc's receding horizon never looks past the end of the plan
    plan.append(plan[-2])
    plan.append(plan[-1])


    # generate reference
    ZMP_ref = generate_zmp_reference(plan, conf.no_mpc_samples_per_step)
    #plot the plan (make sure this workes first)
    print("Plotting the footstep plan and ZMP reference\n")
    planner.plot(simulator)
    
    # setup the lip models
    mpc = LIPMPC(conf)
    
    x0 = np.array([T_support_w.translation[0], 0.0, T_support_w.translation[1], 0.0])
    interpolator = LIPInterpolator(x0, conf)

    # com reference endpoints for the pre-walk lateral sway: from the spawn CoM
    # (centred, tall) to over the first support foot at the walking height
    com_start = robot.stack.comState().value().copy()
    c, c_dot, c_ddot = interpolator.comState()
    com_target = c.copy()
    robot.stack.setComRefState(com_start)
    
    ############################################################################
    # logging
    ############################################################################

    pre_dur = 3.0   # Time to wait befor walking should start

    # Compute number of iterations:
    N_pre = int(round(pre_dur / conf.dt))                          # number of sim steps before walking starts
    N_sim = int(round(no_steps * conf.step_dur / conf.dt))         # total number of sim steps during walking
    N_mpc = int(round(no_steps * conf.step_dur / conf.dt_mpc))     # total number of mpc steps during walking

    # vectors to log all the data of the simulation
    # - COM_POS, COM_VEL, COM_ACC (from the planned reference, pinocchio and pybullet)
    COM_POS_REF = np.nan*np.empty((N_sim, 3))
    COM_VEL_REF = np.nan*np.empty((N_sim, 3))
    COM_ACC_REF = np.nan*np.empty((N_sim, 3))
    COM_POS_PIN = np.nan*np.empty((N_sim, 3))
    COM_VEL_PIN = np.nan*np.empty((N_sim, 3))
    COM_ACC_PIN = np.nan*np.empty((N_sim, 3))
    COM_POS_PB = np.nan*np.empty((N_sim, 3))
    COM_VEL_PB = np.nan*np.empty((N_sim, 3))
    COM_ACC_PB = np.nan*np.empty((N_sim, 3))
    # - Angular momentum (from pinocchio)
    ANG_MOM_PIN = np.nan*np.empty((N_sim, 3))
    # - Left and right foot POS, VEL, ACC (from planned reference, pinocchio)
    LF_POS_REF = np.nan*np.empty((N_sim, 3))
    LF_VEL_REF = np.nan*np.empty((N_sim, 3))
    LF_ACC_REF = np.nan*np.empty((N_sim, 3))
    RF_POS_REF = np.nan*np.empty((N_sim, 3))
    RF_VEL_REF = np.nan*np.empty((N_sim, 3))
    RF_ACC_REF = np.nan*np.empty((N_sim, 3))
    LF_POS_PIN = np.nan*np.empty((N_sim, 3))
    LF_VEL_PIN = np.nan*np.empty((N_sim, 3))
    LF_ACC_PIN = np.nan*np.empty((N_sim, 3))
    RF_POS_PIN = np.nan*np.empty((N_sim, 3))
    RF_VEL_PIN = np.nan*np.empty((N_sim, 3))
    RF_ACC_PIN = np.nan*np.empty((N_sim, 3))
    # - ZMP (from planned reference, from estimator)
    ZMP_REF_LOG = np.nan*np.empty((N_sim, 2))
    ZMP_EST = np.nan*np.empty((N_sim, 2))
    # - DCM (from estimator)
    DCM_EST = np.nan*np.empty((N_sim, 2))
    # - Normal forces in right and left foot (from pybullet ft sensors, from pinocchio)
    FZ_LEFT_PB = np.nan*np.empty(N_sim)
    FZ_RIGHT_PB = np.nan*np.empty(N_sim)
    FZ_LEFT_PIN = np.nan*np.empty(N_sim)
    FZ_RIGHT_PIN = np.nan*np.empty(N_sim)
    TIME = np.nan*np.empty(N_sim)
    
    ############################################################################
    # logging
    ############################################################################
    
    k = 0                                               # current MPC index
    plan_idx = 1                                        # current index of the step within foot step plan
    t_step_elapsed = 0.0                                # elapsed time within current step (use to evaluate spline)
    t_publish = 0.0                                     # last publish time (last time we published something)
    u_k = np.zeros(2)                                   # most recent mpc control (commanded zmp), held between updates
    swing_trajectory = None                             # current swing-foot spline (created at each step transition)
    pin_data = robot.stack.model.createData()           # scratch data for centroidal-momentum queries
    
    for i in range(-N_pre, N_sim):
        t = i * conf.dt
        dt = conf.dt
        
        ########################################################################
        # update the mpc very no_sim_per_mpc steps
        ########################################################################
        
        if i >= 0 and i % conf.no_sim_per_mpc == 0:
            # current LIP state and the zmp reference over the upcoming horizon
            x_k = interpolator.x
            ZMP_ref_k = ZMP_ref[k:k + conf.no_mpc_samples_per_horizon]
            # apply the terminal constraint once the remaining reference runs
            # shorter than a full horizon (the appended duplicate footsteps
            # guarantee this slice never runs out of bounds)
            remaining = len(ZMP_ref) - k
            terminal_idx = remaining - 1 if remaining < conf.no_mpc_samples_per_horizon else -1
            # held until the next mpc update; the interpolator integrates it
            # at sim rate in the per-iteration block below
            u_k = mpc.buildSolveOCP(x_k, ZMP_ref_k, terminal_idx)

            k += 1

        ########################################################################
        # update the foot spline 
        ########################################################################

        if i >= 0 and i % conf.no_sim_per_step == 0:
            next_step = plan[plan_idx]
            robot.setSwingFoot(next_step.side)
            robot.setSupportFoot(other_foot_id(next_step.side))
            T0_swing = robot.swingFootPose()
            T1_swing = next_step.poseInWorld()
            swing_trajectory = SwingFootTrajectory(T0_swing, T1_swing, conf.step_dur)

            t_step_elapsed = 0.0
            plan_idx += 1
            
        ########################################################################
        # in every iteration when walking
        ########################################################################

        if i < 0:
            # pre-walk: smoothly sway the CoM from the spawn pose to over the
            # first support foot (cosine smoothstep -> zero velocity at both ends,
            # so the wbc settles without the overshoot that topples a step input)
            s = (i + N_pre) / N_pre            
            s = 0.5 - 0.5 * np.cos(np.pi * s)     
            robot.stack.setComRefState((1.0 - s) * com_start + s * com_target)

        if i >= 0:
            t_step_elapsed += dt

            # advance the lip model at sim rate with the held mpc control and
            # feed the interpolated com state into the wbc com task
            interpolator.integrate(u_k)
            c, c_dot, c_ddot = interpolator.comState()
            robot.stack.setComRefState(c, c_dot, c_ddot)

            # evaluate the swing-foot spline and update its motion reference
            if swing_trajectory is not None:
                T_swing, V_swing, A_swing = swing_trajectory.evaluate(t_step_elapsed)
                robot.updateSwingFootRef(T_swing, V_swing.vector, A_swing.vector)

        ########################################################################
        # update the simulation
        ########################################################################
        simulator.step()
        robot.update()

        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            robot.publish()

        # store for visualizations
        if i >= 0:
            TIME[i] = t

            # reference (planned) com from the lip interpolator
            COM_POS_REF[i] = c
            COM_VEL_REF[i] = c_dot
            COM_ACC_REF[i] = c_ddot

            # com from the pinocchio / tsid model
            com_state = robot.stack.comState()
            COM_POS_PIN[i] = com_state.value()
            COM_VEL_PIN[i] = com_state.derivative()
            COM_ACC_PIN[i] = com_state.second_derivative()

            # com from pybullet (no direct com acceleration -> left as nan)
            body_id = robot.robot.id()
            COM_POS_PB[i] = simulator.computeComPosition(body_id)
            COM_VEL_PB[i] = simulator.computeComVelocity(body_id)

            # centroidal angular momentum from pinocchio
            q, v = robot.robot.q(), robot.robot.v()
            hg = pin.computeCentroidalMomentum(robot.stack.model, pin_data, q, v)
            ANG_MOM_PIN[i] = hg.angular

            # foot poses / velocities from pinocchio (acc needs the qp dv -> nan)
            H_lf, v_lf = robot.stack.get_pose_vel_acc_LF()
            H_rf, v_rf = robot.stack.get_pose_vel_acc_RF()
            LF_POS_PIN[i] = H_lf.translation
            RF_POS_PIN[i] = H_rf.translation
            LF_VEL_PIN[i] = v_lf.linear
            RF_VEL_PIN[i] = v_rf.linear

            # planned foot reference: the swinging foot follows the spline, the
            # support foot is held fixed at its current placement
            if swing_trajectory is not None:
                T_sw, V_sw, A_sw = swing_trajectory.evaluate(t_step_elapsed)
                if robot.swing_foot == Side.LEFT:
                    LF_POS_REF[i], LF_VEL_REF[i], LF_ACC_REF[i] = T_sw.translation, V_sw.linear, A_sw.linear
                    RF_POS_REF[i], RF_VEL_REF[i], RF_ACC_REF[i] = H_rf.translation, 0.0, 0.0
                else:
                    RF_POS_REF[i], RF_VEL_REF[i], RF_ACC_REF[i] = T_sw.translation, V_sw.linear, A_sw.linear
                    LF_POS_REF[i], LF_VEL_REF[i], LF_ACC_REF[i] = H_lf.translation, 0.0, 0.0

            # zmp: planned (interpolator command) vs estimated
            ZMP_REF_LOG[i] = interpolator.zmp()[:2]
            if robot.zmp is not None:
                ZMP_EST[i] = robot.zmp[:2]
            # dcm: estimated
            if robot.dcm is not None:
                DCM_EST[i] = robot.dcm[:2]

            # normal forces under each foot: pybullet ft sensor vs pinocchio qp
            f_l, _ = robot._foot_wrench_world(conf.lf_frame_name)
            f_r, _ = robot._foot_wrench_world(conf.rf_frame_name)
            FZ_LEFT_PB[i] = f_l[2]
            FZ_RIGHT_PB[i] = f_r[2]
            wr_lf = robot.stack.get_wrench_LF(robot.stack.sol)
            wr_rf = robot.stack.get_wrench_RF(robot.stack.sol)
            FZ_LEFT_PIN[i] = wr_lf[2]
            FZ_RIGHT_PIN[i] = wr_rf[2]


    ########################################################################
    # enough with the simulation, lets plot
    ########################################################################
    
    import matplotlib.pyplot as plt
    try:
        plt.style.use('seaborn-dark')
    except OSError:
        pass  # style was renamed/removed in newer matplotlib
        
    fig, axs = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
    axis_name = ['x', 'y']

    for col in range(2):
        # CoM position
        axs[0, col].plot(TIME, COM_POS_REF[:, col], label='reference (LIP)')
        axs[0, col].plot(TIME, COM_POS_PB[:, col], label='pybullet (truth)')
        axs[0, col].plot(TIME, COM_POS_PIN[:, col], '--', label='pinocchio')
        axs[0, col].set_title(f'CoM position  {axis_name[col]}')
        axs[0, col].set_ylabel('p [m]')

        # CoM velocity
        axs[1, col].plot(TIME, COM_VEL_REF[:, col], label='reference (LIP)')
        axs[1, col].plot(TIME, COM_VEL_PB[:, col], label='pybullet (truth)')
        axs[1, col].plot(TIME, COM_VEL_PIN[:, col], '--', label='pinocchio')
        axs[1, col].set_title(f'CoM velocity  {axis_name[col]}')
        axs[1, col].set_ylabel('v [m/s]')

        # CoM acceleration (no direct pybullet acceleration -> reference vs pinocchio)
        axs[2, col].plot(TIME, COM_ACC_REF[:, col], label='reference (LIP)')
        axs[2, col].plot(TIME, COM_ACC_PIN[:, col], '--', label='pinocchio')
        axs[2, col].set_title(f'CoM acceleration  {axis_name[col]}')
        axs[2, col].set_ylabel('a [m/s^2]')

        # ZMP: planned reference vs estimated from the foot wrenches
        axs[3, col].plot(TIME, ZMP_REF_LOG[:, col], label='reference')
        axs[3, col].plot(TIME, ZMP_EST[:, col], label='estimated')
        axs[3, col].set_title(f'ZMP  {axis_name[col]}')
        axs[3, col].set_ylabel('p_zmp [m]')
        axs[3, col].set_xlabel('t [s]')

    for ax in axs.ravel():
        ax.legend(fontsize=7)

    fig.tight_layout()
    import os
    plots_dir = '/workspaces/leggedrobots/plots/T7'
    os.makedirs(plots_dir, exist_ok=True)
    out_path = os.path.join(plots_dir, 'walking_results.png')
    fig.savefig(out_path, dpi=150)
    print(f"saved plots to {out_path}")
    plt.show()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()