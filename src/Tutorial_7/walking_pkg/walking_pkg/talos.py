import numpy as np
import pinocchio as pin
import pybullet as pb

from scipy.spatial.transform import Rotation as R

# simulator
from simulator.robot import Robot

# whole-body controller
from .tsid_wrapper import TSIDWrapper, create_sample, update_sample

# robot configs
from . import talos_conf as conf

from .footstep_planner import Side

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformBroadcaster


class Talos:
    """Talos robot: combines TSID whole-body controller with PyBullet simulation."""

    def __init__(self, node, simulator):
        self.conf = conf
        self.sim  = simulator
        self.node = node

        ########################################################################
        # whole-body controller
        ########################################################################
        self.stack = TSIDWrapper(conf)

        ########################################################################
        # pybullet robot
        ########################################################################

        model=self.stack.model
        self.robot = Robot(
                    simulator, conf.urdf, model,
                    [0, 0, 0.9], [0, 0, 0, 1],
                    
                    q=conf.q_actuated_home
                )
            
    
        

        ########################################################################
        # state
        ########################################################################
        self.support_foot = Side.RIGHT
        self.swing_foot   = Side.LEFT

        ########################################################################
        # estimators
        ########################################################################
        self.zmp = np.zeros(3)
        self.dcm = np.zeros(3)

        ########################################################################
        # FT sensors
        ########################################################################
        pb.enableJointForceTorqueSensor(
            self.robot.id(),
            self.robot.jointNameIndexMap()["leg_right_6_joint"], True)
        pb.enableJointForceTorqueSensor(
            self.robot.id(),
            self.robot.jointNameIndexMap()["leg_left_6_joint"],  True)

        ########################################################################
        # ROS publishers
        ########################################################################

        # joint states
        self._js_pub = node.create_publisher(JointState, 'joint_states', 10)

        # floating base TF
        self._tf_broadcaster = TransformBroadcaster(node)

        # ZMP and DCM as sphere markers
        self._marker_pub = node.create_publisher(MarkerArray, 'zmp_dcm', 10)

        # foot wrenches
        self._wrench_rf_pub = node.create_publisher(
            WrenchStamped, 'wrench_right_foot', 10)
        self._wrench_lf_pub = node.create_publisher(
            WrenchStamped, 'wrench_left_foot', 10)

    # ──────────────────────────────────────────────────────────────────────────

    def update(self):
        t  = self.sim.time()
        dt = self.sim.dt()

        self.robot.update()
        self._update_zmp_estimate()
        self._update_dcm_estimate()
        self._solve(t, dt)

    # ──────────────────────────────────────────────────────────────────────────
    # foot contact / motion switching
    # ──────────────────────────────────────────────────────────────────────────

    def setSupportFoot(self, side):
        """Put foot on given side into rigid contact and disable its motion task."""
        self.support_foot = side
        if side == Side.RIGHT:
            self.stack.add_contact_RF()
            T = self.stack.get_placement_RF()
            self.stack.set_RF_pose_ref(T, np.zeros(6), np.zeros(6))
        else:
            self.stack.add_contact_LF()
            T = self.stack.get_placement_LF()
            self.stack.set_LF_pose_ref(T, np.zeros(6), np.zeros(6))

    def setSwingFoot(self, side):
        """Release contact on given side and enable its motion task."""
        self.swing_foot = side
        if side == Side.RIGHT:
            self.stack.remove_contact_RF()
        else:
            self.stack.remove_contact_LF()

    def updateSwingFootRef(self, T_swing_w, V_swing_w, A_swing_w):
        """Set swing foot pose, velocity and acceleration reference.

        Args:
            T_swing_w (pin.SE3):    target pose
            V_swing_w (np.array 3): linear velocity
            A_swing_w (np.array 3): linear acceleration
        """
        # pad 3D linear to 6D spatial (angular part = 0)
        vel_6 = np.zeros(6); vel_6[3:] = V_swing_w
        acc_6 = np.zeros(6); acc_6[3:] = A_swing_w

        if self.swing_foot == Side.RIGHT:
            self.stack.set_RF_pose_ref(T_swing_w, vel_6, acc_6)
        else:
            self.stack.set_LF_pose_ref(T_swing_w, vel_6, acc_6)

    def swingFootPose(self):
        """Return current swing foot SE3 pose."""
        if self.swing_foot == Side.RIGHT:
            return self.stack.get_placement_RF()
        return self.stack.get_placement_LF()

    def supportFootPose(self):
        """Return current support foot SE3 pose."""
        if self.support_foot == Side.RIGHT:
            return self.stack.get_placement_RF()
        return self.stack.get_placement_LF()

    # ──────────────────────────────────────────────────────────────────────────
    # ROS publishing
    # ──────────────────────────────────────────────────────────────────────────

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()

        # joint states
        js          = JointState()
        js.header.stamp = stamp
        js.name     = list(self.robot.actuatedJointNames())
        js.position = list(self.robot.actuatedJointPosition())
        js.velocity = list(self.robot.actuatedJointVelocity())
        self._js_pub.publish(js)

        # floating base TF
        T_base = self.stack.baseState()[0]
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id  = 'base_link'
        tf_msg.transform.translation.x = float(T_base.translation[0])
        tf_msg.transform.translation.y = float(T_base.translation[1])
        tf_msg.transform.translation.z = float(T_base.translation[2])
        quat = R.from_matrix(T_base.rotation).as_quat()
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self._tf_broadcaster.sendTransform(tf_msg)

        # ZMP and DCM sphere markers
        ma = MarkerArray()
        for idx, (point, color, ns) in enumerate([
            (self.zmp, (1., 0., 0.), 'zmp'),
            (self.dcm, (0., 0., 1.), 'dcm'),
        ]):
            m             = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'world'
            m.ns     = ns
            m.id     = idx
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(point[0])
            m.pose.position.y = float(point[1])
            m.pose.position.z = float(point[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = *color, 1.0
            ma.markers.append(m)
        self._marker_pub.publish(ma)

        # foot wrenches
        self._publish_wrench(
            self._wrench_rf_pub, stamp,
            self.stack.get_wrench_RF(self.stack.sol), 'right_sole')
        self._publish_wrench(
            self._wrench_lf_pub, stamp,
            self.stack.get_wrench_LF(self.stack.sol), 'left_sole')

    # ──────────────────────────────────────────────────────────────────────────
    # private
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_wrench(self, pub, stamp, wrench_6, frame_id):
        msg = WrenchStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.wrench.force.x  = float(wrench_6[0])
        msg.wrench.force.y  = float(wrench_6[1])
        msg.wrench.force.z  = float(wrench_6[2])
        msg.wrench.torque.x = float(wrench_6[3])
        msg.wrench.torque.y = float(wrench_6[4])
        msg.wrench.torque.z = float(wrench_6[5])
        pub.publish(msg)

    def _solve(self, t, dt):
        q   = self.robot.q()
        v   = self.robot.v()
        tau, _ = self.stack.update(q, v, t)
        self.robot.sendTorques(tau)

    def _update_zmp_estimate(self):
        """Estimate ZMP from foot FT sensors using weighted average."""
        q    = self.robot.q()
        data = self.stack.robot.model().createData()
        pin.framesForwardKinematics(self.stack.robot.model(), data, q)

        def read_ankle_wrench(joint_name):
            raw = pb.getJointState(
                self.robot.id(),
                self.robot.jointNameIndexMap()[joint_name])[2]
            return pin.Force(np.array(
                [-raw[0], -raw[1], -raw[2], -raw[3], -raw[4], -raw[5]]))

        wr = read_ankle_wrench("leg_right_6_joint")
        wl = read_ankle_wrench("leg_left_6_joint")

        fz_r     = wr.linear[2]
        fz_l     = wl.linear[2]
        fz_total = fz_r + fz_l
        if abs(fz_total) < 1.0:
            return

        model = self.stack.robot.model()
        H_r = data.oMf[model.getFrameId(conf.rf_frame_name)]
        H_l = data.oMf[model.getFrameId(conf.lf_frame_name)]

        def foot_zmp(H, w):
            """ZMP from ankle wrench in world frame."""
            f_w   = H.rotation @ w.linear
            tau_w = H.rotation @ w.angular
            p     = H.translation
            if abs(f_w[2]) < 1.0:
                return np.zeros(3)
            zmp_x = p[0] - (tau_w[1] + f_w[0] * p[2]) / f_w[2]
            zmp_y = p[1] + (tau_w[0] - f_w[1] * p[2]) / f_w[2]
            return np.array([zmp_x, zmp_y, 0.0])

        zmp_r    = foot_zmp(H_r, wr)
        zmp_l    = foot_zmp(H_l, wl)
        self.zmp = (fz_r * zmp_r + fz_l * zmp_l) / fz_total

    def _update_dcm_estimate(self):
        """Estimate DCM from pinocchio CoM state: ξ = c + ċ/ω₀."""
        data  = self.stack.formulation.data()
        c     = self.stack.robot.com(data)
        c_dot = self.stack.robot.com_vel(data)
        omega = np.sqrt(conf.g / conf.h)
        self.dcm = c + c_dot / omega