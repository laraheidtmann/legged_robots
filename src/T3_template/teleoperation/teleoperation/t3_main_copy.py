import numpy as np

import numpy.linalg as la


from simulator.pybullet_wrapper import PybulletWrapper

from simulator.robot import Robot


import pinocchio as pin

from pinocchio.robot_wrapper import RobotWrapper


from enum import Enum

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped

from scipy.interpolate import CubicSpline


import rclpy

from rclpy.node import Node

from sensor_msgs.msg import JointState

from geometry_msgs.msg import PoseStamped


################################################################################

# Robot

################################################################################


class Talos(Robot):


    def __init__(self, simulator, useFixedBase=True):


        urdf = "src/talos_description/robots/talos_reduced.urdf"

        path_meshes = "src/talos_description/meshes/../.."

        z_init = 1.15


        q_actuated_home = np.zeros(32)


        self._wrapper = pin.RobotWrapper.BuildFromURDF(

            urdf, path_meshes, None, True, None

        )

        self.model = self._wrapper.model


        super().__init__(

            simulator, urdf, self.model,

            [0, 0, z_init], [0, 0, 0, 1],

            q=q_actuated_home, useFixedBase=useFixedBase

        )


        self.node = rclpy.create_node("teleoperator")

        self.joint_state_publisher = self.node.create_publisher(JointState, "joint_states", 10)

        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.marker_listener = self.node.create_subscription(

            PoseStamped, 'marker_pose', self._marker_callback, 10

        )

        self.X_Goal = None


    def _marker_callback(self, msg):

        p = msg.pose.position

        q = msg.pose.orientation

        self.X_Goal = pin.XYZQUATToSE3([p.x, p.y, p.z, q.x, q.y, q.z, q.w])


    def update(self):

        super().update()

        # Update with velocity but zero acceleration — only for frame placements

        pin.forwardKinematics(self._wrapper.model, self._wrapper.data, self.q(), self.v())

        pin.updateFramePlacements(self._wrapper.model, self._wrapper.data)


    def update_goal(self):

        """Call this every frame during joint spline to keep X_Goal fresh."""

        # Explicit clean FK with position only for a stable goal pose

        pin.forwardKinematics(self._wrapper.model, self._wrapper.data, self.q())

        pin.updateFramePlacements(self._wrapper.model, self._wrapper.data)

        hand_id = self._wrapper.model.getJointId("arm_right_7_joint")

        self.X_Goal = self._wrapper.data.oMi[hand_id].copy()


    def publish(self):

        msg = JointState()

        msg.header.stamp = self.node.get_clock().now().to_msg()

        msg.name = self.actuatedJointNames()

        msg.position = list(self.actuatedJointPosition())

        msg.velocity = list(self.actuatedJointVelocity())

        self.joint_state_publisher.publish(msg)


        # base_link TF

        tf = TransformStamped()

        tf.header.stamp = msg.header.stamp

        tf.header.frame_id = "world"

        tf.child_frame_id = "base_link"

        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)


        # hand TF

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


class JointSpaceController:

    def __init__(self, robot, Kp, Kd):

        self.Kp = Kp

        self.Kd = Kd

        self.robot = robot


    def update(self, q_r, q_r_dot, q_r_ddot):

        q = self.robot.q()

        v = self.robot.v()

        model = self.robot.wrapper().model

        data = self.robot.wrapper().data

        h = pin.rnea(model, data, q, v, np.zeros(model.nv))

        M = pin.crba(model, data, q)

        M = (M + M.T) / 2.0

        return M @ (q_r_ddot - self.Kd @ (v - q_r_dot) - self.Kp @ (q - q_r)) + h



class CartesianSpaceController:

    def __init__(self, robot, joint_name, Kp, Kd):

        self.robot = robot

        self.joint_name = joint_name

        self.Kp = Kp

        self.Kd = Kd

        self.joint_id = self.robot.wrapper().model.getJointId(joint_name)

        self.damp = 1e-3


    def update(self, X_r, X_dot_r, X_ddot_r):

        q = self.robot.q()

        v = self.robot.v()

        model = self.robot.wrapper().model

        data = self.robot.wrapper().data


        # FK with zero acceleration to get Jdot*v correctly

        pin.forwardKinematics(model, data, q, v, np.zeros(model.nv))

        pin.computeJointJacobians(model, data, q)

        pin.updateFramePlacements(model, data)


        # Current pose

        X = data.oMi[self.joint_id]


        # Pose error

        iMd = X.actInv(X_r)

        err = pin.log(iMd).vector


        # Jacobian

        J = pin.computeJointJacobian(model, data, q, self.joint_id)


        # Cartesian velocity
        X_dot = J @ v

        # Bias acceleration Jdot*qdot

        Jdot_v = pin.getClassicalAcceleration(

            model, data, self.joint_id, pin.ReferenceFrame.LOCAL

        ).vector


        # Desired Cartesian acceleration

        X_ddot_des = X_ddot_r - self.Kd @ (X_dot - X_dot_r) + self.Kp @ err


        # Damped pseudo-inverse

        J_pinv = J.T @ np.linalg.inv(J @ J.T + self.damp * np.eye(6))


        # Joint acceleration

        q_ddot_des = J_pinv @ (X_ddot_des - Jdot_v)


        # Inverse dynamics

        M = pin.crba(model, data, q)

        M = (M + M.T) / 2.0

        h = pin.rnea(model, data, q, v, np.zeros(model.nv))


        tau_cart = M @ q_ddot_des + h


        # Nullspace projector N = I - J^T (J#)^T

        N = np.eye(model.nv) - J.T @ J_pinv.T


        return tau_cart, N, J, J_pinv



################################################################################

# Spline

################################################################################

class JointSpline:
    def __init__(self, q_init, q_goal, duration):
        self.duration = duration
        t_points = [0.0, duration]
        q_points = np.vstack([q_init, q_goal])
        self.spline = CubicSpline(
            t_points, q_points, axis=0,
            bc_type=((1, np.zeros(len(q_init))), (1, np.zeros(len(q_goal))))
        )

    def evaluate(self, t):
        t = np.clip(t, 0.0, self.duration)
        return self.spline(t), self.spline(t, 1), self.spline(t, 2)



################################################################################
# Main
################################################################################


def main():
    rclpy.init()

    simulator = PybulletWrapper()
    robot = Talos(simulator)

    n_joints = robot.model.nv  # 32


    # ── Joint space gains ────────────────────────────────────────────────────

    Kp_diag = np.ones(n_joints) * 20.0
    Kd_diag = np.ones(n_joints) * 2.0

    Kp_diag[0:12] = 300.0
    Kd_diag[0:12] = 10.0
    Kp_diag[12:] = 20.0
    Kd_diag[12:] = 2.0
    Kp = np.diag(Kp_diag)
    Kd = np.diag(Kd_diag)


    # ── Cartesian gains ──────────────────────────────────────────────────────
    Kp_cart = np.eye(6) * 100.0   # much higher than before
    Kd_cart = np.eye(6) * 10.0


    # ── Posture (nullspace) gains — soft, so they don't fight Cartesian ──────
    Kp_posture_diag = np.ones(n_joints) * 10.0
    Kd_posture_diag = np.ones(n_joints) * 1.0

    # legs/torso slightly stiffer to resist gravity drift
    Kp_posture_diag[0:12] = 20.0
    Kd_posture_diag[0:12] = 2.0

    # arms softer — let Cartesian task dominate

    Kp_posture_diag[12:] = 40.0
    Kd_posture_diag[12:] = 1
    Kp_posture = np.diag(Kp_posture_diag)
    Kd_posture = np.diag(Kd_posture_diag)


    # ── Home configuration ───────────────────────────────────────────────────

    q_init = robot.q().copy()   # size = nv = 32
    q_home = q_init.copy()
    q_home[0:6]   = np.array([0, 0, 0, 0, 0, 0])    # left leg
    q_home[6:12]  = np.array([0, 0, 0, 0, 0, 0])    # right leg
    q_home[14:22] = np.array([0, -0.24, 0, -1, 0, 0, 0, 0])  # left arm
    q_home[22:30] = np.array([0, -0.24, 0, -1, 0, 0, 0, 0])  # right arm

    spline = JointSpline(q_init, q_home, 5.0)

    joint_space_controller = JointSpaceController(robot, Kp, Kd)
    cartesian_space_controller = CartesianSpaceController(
        robot, "arm_right_7_joint", Kp_cart, Kd_cart
    )


    while rclpy.ok():

        # ── Update state ─────────────────────────────────────────────────────
        robot.update()
        rclpy.spin_once(robot.node, timeout_sec=0)
        robot.publish()
        t = simulator.simTime()
        if t < 5.0:
            # Joint spline phase
            q_r, q_r_dot, q_r_ddot = spline.evaluate(t)
            tau = joint_space_controller.update(q_r, q_r_dot, q_r_ddot)
            # Keep X_Goal fresh with a clean FK every frame
            robot.update_goal()
        else:
            # Cartesian control phase
            # Use marker goal if available, else hold end-of-spline pose
            X_r = robot.X_Goal
            tau_cart, N, J, J_pinv = cartesian_space_controller.update(
                X_r, np.zeros(6), np.zeros(6)
            )
            # Posture task in nullspace — simple PD, no dynamics
            q = robot.q()
            v = robot.v()
            tau_posture = -Kp_posture @ (q - q_home) - Kd_posture @ v
            tau = tau_cart + N @ tau_posture

        robot.setActuatedJointTorques(tau)
        simulator.debug()
        simulator.step()



if __name__ == '__main__':
    main()
