import numpy as np
import pinocchio as pin
import pybullet as pb

# simulator
from simulator.robot import Robot
# whole-body controller
from .tsid_wrapper_flo import TSIDWrapper
# robot configs
from . import talos_conf as conf
from .footstep_planner import Side

# ROS visualizations
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

class Talos:
    """Talos robot
    combines wbc with pybullet, functions to read and set
    sensor values.
    """
    def __init__(self, simulator):
        self.conf = conf
        self.sim = simulator

        # setup tsid wrapper for whole body controller
        # stack = stack of tasks in TSID framework
        self.stack = TSIDWrapper(self.conf)

        # spawn robot in simulation
        self.robot = Robot(
            self.sim,
            self.conf.urdf,
            self.stack.model,
            basePosition=[0, 0, 1.1],
            baseQuationerion=[0, 0, 0, 1],
            useFixedBase=False,
            verbose=True,
        )

        ########################################################################
        # state
        ########################################################################
        self.support_foot = Side.RIGHT
        self.swing_foot = Side.LEFT

        ########################################################################
        # estimators
        ########################################################################
        self.zmp = None
        self.dcm = None

        ########################################################################
        # sensors
        ########################################################################
        self._ft_joint = {
            self.conf.rf_frame_name: "leg_right_6_joint",
            self.conf.lf_frame_name: "leg_left_6_joint",
        }
        for ankle_joint in self._ft_joint.values():
            pb.enableJointForceTorqueSensor(
                self.robot.id(), self.robot.jointNameIndexMap()[ankle_joint], True
            )
        ########################################################################
        # visualizations
        ########################################################################
        self._node = rclpy.create_node("walking_control_node")
        self._joint_names = list(self.robot.actuatedJointNames())
        self._tf_broadcaster = TransformBroadcaster(self._node)

        self._joint_pub = self._node.create_publisher(JointState, 'joint_states', 10)
        self._odom_pub = self._node.create_publisher(Odometry, 'odom', 10)
        self._zmp_pub = self._node.create_publisher(MarkerArray, 'zmp', 10)
        self._wrench_rf_pub = self._node.create_publisher(WrenchStamped, 'wrench_rf', 10)
        self._wrench_lf_pub = self._node.create_publisher(WrenchStamped, 'wrench_lf', 10)
           
    def update(self):
        """updates the robot
        """
        t = self.sim.simTime()
        dt = self.sim.stepTime()

        self.robot.update()

        # update the estimators
        self._update_zmp_estimate()
        self._update_dcm_estimate()
        
        # update wbc and send back to pybullet
        self._solve(t, dt)
        
    def setSupportFoot(self, side):
        """sets the the support foot of the robot on given side
        """
        self.support_foot = side

        if side == Side.RIGHT:
            self.stack.add_contact_RF()
            self.stack.remove_motion_RF()
        elif side == Side.LEFT:
            self.stack.add_contact_LF()
            self.stack.remove_motion_LF()
        else: 
            raise ValueError("Invalid side for Support foot: {}".format(side))


    def setSwingFoot(self, side):
        """sets the swing foot of the robot on given side
        """
        self.swing_foot = side
        
        if side == Side.RIGHT:
            self.stack.remove_contact_RF()
            self.stack.add_motion_RF()
        elif side == Side.LEFT:
            self.stack.remove_contact_LF()
            self.stack.add_motion_LF()
        else:
            raise ValueError("Invalid side for swing foot: {}".format(side))
        
    def updateSwingFootRef(self, T_swing_w, V_swing_w, A_swing_w):
        """updates the swing foot motion reference
        """

        if self.swing_foot == Side.RIGHT:
            self.stack.set_RF_pose_ref(T_swing_w, V_swing_w, A_swing_w)
        elif self.swing_foot == Side.LEFT:
            self.stack.set_LF_pose_ref(T_swing_w, V_swing_w, A_swing_w)
        else:
            raise ValueError("Invalid side for swing foot: {}".format(self.swing_foot))


    def swingFootPose(self):
        """return the pose of the current swing foot
        """
        if self.swing_foot == Side.RIGHT:
            return self.stack.get_placement_RF()
        elif self.swing_foot == Side.LEFT:
            return self.stack.get_placement_LF()
        else:
            raise ValueError("Invalid side for swing foot: {}".format(self.swing_foot))

    def supportFootPose(self):
        """return the pose of the current support foot
        """
        if self.support_foot == Side.RIGHT:
            return self.stack.get_placement_RF()
        elif self.support_foot == Side.LEFT:
            return self.stack.get_placement_LF()
        else:
            raise ValueError("Invalid side for support foot: {}".format(self.support_foot))

    def publish(self):
        now = self._node.get_clock().now().to_msg()

        # joint state
        msg = JointState()
        msg.header.stamp = now
        msg.name = self._joint_names
        msg.position = self.robot.actuatedJointPosition().tolist()
        msg.velocity = self.robot.actuatedJointVelocity().tolist()
        self._joint_pub.publish(msg)

        # odometry (base pose + velocity)
        T_b_w, V_b_w = self.stack.baseState()
        xyzquat = pin.SE3ToXYZQUAT(T_b_w)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = xyzquat[0]
        odom.pose.pose.position.y = xyzquat[1]
        odom.pose.pose.position.z = xyzquat[2]
        odom.pose.pose.orientation.x = xyzquat[3]
        odom.pose.pose.orientation.y = xyzquat[4]
        odom.pose.pose.orientation.z = xyzquat[5]
        odom.pose.pose.orientation.w = xyzquat[6]
        odom.twist.twist.linear.x = V_b_w.linear[0]
        odom.twist.twist.linear.y = V_b_w.linear[1]
        odom.twist.twist.linear.z = V_b_w.linear[2]
        odom.twist.twist.angular.x = V_b_w.angular[0]
        odom.twist.twist.angular.y = V_b_w.angular[1]
        odom.twist.twist.angular.z = V_b_w.angular[2]
        self._odom_pub.publish(odom)
        self._tf_broadcaster.sendTransform(self._odomToTransform(odom))

        # feet wrenches
        wr_rf = self.stack.get_wrench_RF(self.stack.sol)
        wr_lf = self.stack.get_wrench_LF(self.stack.sol)
        self._publish_wrench(self._wrench_rf_pub, now, self.conf.rf_frame_name, wr_rf)
        self._publish_wrench(self._wrench_lf_pub, now, self.conf.lf_frame_name, wr_lf)

        # zmp and dcm markers
        markers = MarkerArray()
        if self.zmp is not None:
            markers.markers.append(self._point_marker(now, 0, self.zmp, [1.0, 0.0, 0.0, 1.0]))
        if self.dcm is not None:
            markers.markers.append(self._point_marker(now, 1, self.dcm, [0.0, 0.0, 1.0, 1.0]))
        self._zmp_pub.publish(markers)

    ############################################################################
    # private funcitons
    ############################################################################

    def _odomToTransform(self, odom):
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = odom.header.frame_id
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        return tf

    def _publish_wrench(self, pub, stamp, frame_id, wrench):
        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.wrench.force.x = wrench[0]
        msg.wrench.force.y = wrench[1]
        msg.wrench.force.z = wrench[2]
        msg.wrench.torque.x = wrench[3]
        msg.wrench.torque.y = wrench[4]
        msg.wrench.torque.z = wrench[5]
        pub.publish(msg)

    def _point_marker(self, stamp, marker_id, position, rgba):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "ground_reference_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _solve(self, t, dt):
        # get the current state
        q = self.robot.q()
        v = self.robot.v()
        
        # solve the whole body qp
        tau, dv = self.stack.update(q, v, t)
        self.robot.setActuatedJointTorques(tau)

    def _foot_wrench_world(self, frame_name):
        """reads the foot FT sensor (mounted on the ankle joint) and returns
        (force, torque) expressed about the world origin
        """
        # the sole_fix joint carries no force; read the ankle joint instead
        joint_id = self.robot.jointNameIndexMap()[self._ft_joint[frame_name]]
        # PyBullet reports the constraint reaction wrench, negate to get the contact wrench
        wrench_local = -np.array(pb.getJointState(self.robot.id(), joint_id)[2])
        ls = pb.getLinkState(self.robot.id(), joint_id, computeForwardKinematics=True)
        H_w_ankle = pin.XYZQUATToSE3(np.array(list(ls[0]) + list(ls[1])))
        # act() transfers the moment to the world origin, so the downstream ZMP
        # formula uses d=0 (the ankle height is absorbed here)
        wrench_w = H_w_ankle.act(pin.Force(wrench_local))
        return wrench_w.linear, wrench_w.angular

    def _zmp_single_foot(self, force, torque, f_min=1.0):
        """ZMP under a single foot, given its world-frame force/torque at the sole plane (d=0)
        """
        if force[2] < f_min:
            return np.zeros(3)
        p_x = (-torque[1]) / force[2]
        p_y = (torque[0]) / force[2]
        return np.array([p_x, p_y, 0.0])

    def _update_zmp_estimate(self):
        """update the estimated zmp position
        """
        f_r, tau_r = self._foot_wrench_world(self.conf.rf_frame_name)
        f_l, tau_l = self._foot_wrench_world(self.conf.lf_frame_name)

        zmp_r = self._zmp_single_foot(f_r, tau_r)
        zmp_l = self._zmp_single_foot(f_l, tau_l)

        f_z_total = f_r[2] + f_l[2]
        if f_z_total < 10.0:
            self.zmp = np.zeros(3)
        else:
            self.zmp = (zmp_r * f_r[2] + zmp_l * f_l[2]) / f_z_total

    def _update_dcm_estimate(self):
        """update the estimated capture point / DCM based on current center of mass state
        """
        com_state = self.stack.comState()
        com_pos = com_state.value()
        com_vel = com_state.derivative()

        omega = np.sqrt(9.81 / com_pos[2])
        self.dcm = np.array([
            com_pos[0] + com_vel[0] / omega,
            com_pos[1] + com_vel[1] / omega,
            0.0,
        ])