#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import numpy as np

import rospy
import tf
from tf.transformations import euler_from_quaternion

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import matrix2rpy
from skrobot.model import Link
from skrobot.model.joint import FloatingJoint
from skrobot.models.urdf import RobotModelFromURDF

from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped


URDF_PATH = "/home/tokunaga/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/hydrus/robots/quad/tilt_0deg_ce_15inch_202604/robot.urdf"


class CeilingCircleIKNode(object):
    def __init__(self):
        rospy.init_node("ceiling_circle_ik_node")

        self.args = self.parse_args()

        self.load_params()
        self.setup_ros()
        self.setup_robot_model()
        self.setup_viewer()
        self.init_state()

    # ============================================================
    # initialization
    # ============================================================

    def parse_args(self):
        parser = argparse.ArgumentParser()

        parser.add_argument("--steps", type=int, default=10000)
        parser.add_argument("--radius", type=float, default=0.15)
        parser.add_argument("--control-hz", type=float, default=50.0)
        parser.add_argument("--ik-stop", type=int, default=20)
        parser.add_argument("--fix-joint", type=str, default="joint2",
                            choices=["joint1", "joint2", "joint3"])
        parser.add_argument("--fix-angle", type=float, default=0.6)
        parser.add_argument("--q1-center", type=float, default=1.0)
        parser.add_argument("--q3-center", type=float, default=1.0)
        parser.add_argument("--max-cog-step", type=float, default=0.0001)
        parser.add_argument("--max-q-step", type=float, default=0.001)
        parser.add_argument("--resolution", type=int, nargs=2, default=(960, 720))
        parser.add_argument("--update-interval", type=float, default=0.02)
        parser.add_argument("--trail-points", type=int, default=80)

        return parser.parse_args(rospy.myargv()[1:])

    def load_params(self):
        # load hydrus namespace
        self.robot_ns = rospy.get_param("~robot_ns", "/hydrus")
        if not self.robot_ns.startswith("/"):
            self.robot_ns = "/" + self.robot_ns
        self.robot_ns = self.robot_ns.rstrip("/")

        # ceiling / altitude params
        self.target_d_R = rospy.get_param(
            "~target_d_R",
            rospy.get_param(self.robot_ns + "/target_d_R", 5.0)
        )
        self.ceiling_height = rospy.get_param(self.robot_ns + "/ceiling_height")
        self.ceiling_distance_offset = rospy.get_param(
            self.robot_ns + "/ceiling_distance_offset"
        )
        self.rotor_radius = rospy.get_param(self.robot_ns + "/rotor_radius")
        self.target_d = self.target_d_R * self.rotor_radius
        self.target_z = (
            self.ceiling_height
            - self.ceiling_distance_offset
            - self.target_d
        )

        # altitude stability
        self.z_threshold = rospy.get_param("~z_threshold", 0.10)
        self.vz_threshold = rospy.get_param("~vz_threshold", 0.10)
        self.stable_time = rospy.get_param("~stable_time", 1.0)

        # z velocity control
        self.kp_z = rospy.get_param("~kp_z", 0.10)
        self.max_vz = rospy.get_param("~max_vz", 0.05)

        # start pose stability
        self.start_err_tol = rospy.get_param("~start_err_tol", 0.07)
        self.radius_err_tol = rospy.get_param("~radius_err_tol", 0.07)
        self.start_stable_time = rospy.get_param("~start_stable_time", 1.0)

    def setup_ros(self):
        self.tf_listener = tf.TransformListener()

        self.nav_pub = rospy.Publisher(
            self.robot_ns + "/uav/nav",
            FlightNav,
            queue_size=1
        )

        self.joint_pub = rospy.Publisher(
            self.robot_ns + "/joints_ctrl",
            JointState,
            queue_size=1
        )

        self.joint_sub = rospy.Subscriber(
            self.robot_ns + "/joint_states",
            JointState,
            self.joint_state_callback,
            queue_size=1
        )

        self.odom_sub = rospy.Subscriber(
            self.robot_ns + "/uav/baselink/odom",
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        self.debug_pubs = {
            "leg5": rospy.Publisher("/debug/leg5", Vector3Stamped, queue_size=1),
            "target_leg5": rospy.Publisher("/debug/target_leg5", Vector3Stamped, queue_size=1),
            "leg5_error": rospy.Publisher("/debug/leg5_error", Vector3Stamped, queue_size=1),
            "root": rospy.Publisher("/debug/root", Vector3Stamped, queue_size=1),
            "target_root": rospy.Publisher("/debug/target_root", Vector3Stamped, queue_size=1),
            "root_error": rospy.Publisher("/debug/root_error", Vector3Stamped, queue_size=1),
        }

        self.current_q = None
        self.current_z = None
        self.current_vz = None
        self.current_root_z = None

    def setup_robot_model(self):
        rospy.sleep(1.0)

        robot_description_param = self.robot_ns + "/robot_description_rviz"
        rospy.loginfo("Loading URDF from param: %s", robot_description_param)
        self.robot = RobotModelFromURDF.from_robot_description(
            robot_description_param
        )
        self.fixed_root_xyz, self.root_rpy0 = self.get_xyz_rpy(
            "world",
            self.robot_ns.lstrip("/") + "/root"
        )
        self.attach_world_floating_base()
        self.set_joint_limits()
        self.set_fixed_joint()
        self.setup_ik_links()
        self.leg5 = self.robot.leg5
        self.compute_alpha0()
        # initial command
        self.q_cmd = self.get_actual_q()
        self.cog_cmd, _ = self.get_xyz_rpy(
            "world",
            self.robot_ns.lstrip("/") + "/cog"
        )
        rospy.loginfo("fixed_root_xyz = %s", self.fixed_root_xyz)
        rospy.loginfo("target_z = %.3f", self.target_z)

    def setup_viewer(self):
        self.viewer = skrobot.viewers.PyrenderViewer(
            resolution=tuple(self.args.resolution),
            update_interval=self.args.update_interval
        )
        self.viewer.add(self.robot)

        self.trail_spheres = []
        self.target_axis = None
        self.root_axis = None

        from skrobot.model.primitives import Sphere

        dummy_center = self.leg5.worldpos().copy()

        for j in range(self.args.trail_points):
            phi = 2.0 * np.pi * j / self.args.trail_points
            wp = dummy_center + np.array([
                self.args.radius * np.cos(phi),
                self.args.radius * np.sin(phi),
                0.0
            ])
            mark = Sphere(radius=0.008, pos=wp)
            mark.set_color([60, 120, 220, 255])
            self.viewer.add(mark)
            self.trail_spheres.append(mark)

        self.target_axis = skrobot.model.Axis(
            axis_radius=0.015,
            axis_length=0.18,
            pos=dummy_center.copy()
        )
        self.viewer.add(self.target_axis)

        self.root_axis = skrobot.model.Axis(
            axis_radius=0.008,
            axis_length=0.20,
            pos=self.fixed_root_xyz.copy()
        )
        self.viewer.add(self.root_axis)

        self.viewer.show()

    def init_state(self):
        self.state = "WAIT_READY"
        self.state_start_time = rospy.Time.now()

        self.altitude_stable_start_time = None
        self.start_pose_stable_start_time = None

        self.center_world = None
        self.k = 0
        self.errors = []
        
        self.yaw_cmd = 0.0

    # ============================================================
    # callbacks
    # ============================================================

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        if (
            "joint1" in name_to_pos and
            "joint2" in name_to_pos and
            "joint3" in name_to_pos
        ):
            self.current_q = np.array([
                name_to_pos["joint1"],
                name_to_pos["joint2"],
                name_to_pos["joint3"]
            ], dtype=float)

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z
        self.current_vz = msg.twist.twist.linear.z

    # ============================================================
    # robot model setup
    # ============================================================

    def attach_world_floating_base(self):
        world_link = Link(name="world")

        self.fjoint = FloatingJoint(
            parent_link=world_link,
            child_link=self.robot.root_link,
            name="world_to_root"
        )

        self.robot.root_link._parent_link = world_link
        world_link.add_child_link(self.robot.root_link)
        self.robot.root_link.joint = self.fjoint

        self.fjoint.joint_angle(
            np.r_[self.fixed_root_xyz, self.root_rpy0]
        )

    def set_joint_limits(self):
        self.robot.joint1.min_angle = 0.3
        self.robot.joint1.max_angle = 1.47

        self.robot.joint3.min_angle = 0.3
        self.robot.joint3.max_angle = 1.47

    def set_fixed_joint(self):
        fixed_joint = getattr(self.robot, self.args.fix_joint)
        fixed_joint.joint_angle(self.args.fix_angle)

    def setup_ik_links(self):
        joint_to_child_link = {
            "joint1": self.robot.link2,
            "joint2": self.robot.link3,
            "joint3": self.robot.link4,
        }

        self.link_list = [
            link for jname, link in joint_to_child_link.items()
            if jname != self.args.fix_joint
        ]

        rospy.loginfo(
            "IK link_list = %s",
            [link.name for link in self.link_list]
        )

    def compute_alpha0(self):
        self.robot.joint1.joint_angle(self.args.q1_center)
        self.robot.joint2.joint_angle(self.args.fix_angle)
        self.robot.joint3.joint_angle(self.args.q3_center)

        self.fjoint.joint_angle(
            np.r_[self.fixed_root_xyz, 0.0, 0.0, 0.0]
        )

        leg5_vec = self.robot.leg5.worldpos()[:2] - self.fixed_root_xyz[:2]
        self.alpha0 = np.arctan2(leg5_vec[1], leg5_vec[0])

        rospy.loginfo("alpha0 = %.3f", self.alpha0)

    # ============================================================
    # common utilities
    # ============================================================

    def change_state(self, next_state):
        rospy.loginfo("state: %s -> %s", self.state, next_state)
        self.state = next_state
        self.state_start_time = rospy.Time.now()

    def clamp(self, x, min_value, max_value):
        return max(min(x, max_value), min_value)

    def get_xyz_rpy(self, parent_frame, child_frame):
        trans, quat = self.tf_listener.lookupTransform(
            parent_frame,
            child_frame,
            rospy.Time(0)
        )
        xyz = np.array(trans, dtype=float)
        rpy = np.array(euler_from_quaternion(quat), dtype=float)
        return xyz, rpy

    def update_root_z_from_tf(self):
        try:
            root_xyz, _ = self.get_xyz_rpy(
                "world",
                self.robot_ns.lstrip("/") + "/root"
            )
            self.current_root_z = root_xyz[2]
            return True
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF: world -> %s/root",
                self.robot_ns.lstrip("/")
            )
            return False

    def get_actual_q(self):
        if self.current_q is None:
            return np.array([
                self.args.q1_center,
                self.args.fix_angle,
                self.args.q3_center
            ], dtype=float)

        return self.current_q.copy()

    def limit_vector_step(self, current, target, max_step):
        diff = target - current
        norm = np.linalg.norm(diff)

        if norm < max_step or norm < 1e-9:
            return target.copy()

        return current + diff / norm * max_step

    def limit_joint_step(self, current, target, max_step):
        diff = target - current
        diff_limited = np.clip(diff, -max_step, max_step)
        return current + diff_limited

    # ============================================================
    # altitude control
    # ============================================================

    def publish_z_velocity_command(self):
        if self.current_root_z is None:
            return

        error_z = self.target_z - self.current_root_z
        vz_cmd = self.kp_z * error_z
        vz_cmd = self.clamp(vz_cmd, -self.max_vz, self.max_vz)

        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_z_nav_mode = FlightNav.VEL_MODE
        msg.target_vel_z = vz_cmd

        self.nav_pub.publish(msg)

        rospy.loginfo_throttle(
            1.0,
            "[GO_TARGET_ALTITUDE] root_z=%.3f, target_z=%.3f, vz_cmd=%.3f",
            self.current_root_z,
            self.target_z,
            vz_cmd
        )

    def is_altitude_stable(self):
        if self.current_root_z is None or self.current_vz is None:
            return False

        z_error = abs(self.target_z - self.current_root_z)
        vz_abs = abs(self.current_vz)

        return z_error < self.z_threshold and vz_abs < self.vz_threshold

    def update_altitude_stable_timer(self):
        now = rospy.Time.now()

        if self.is_altitude_stable():
            if self.altitude_stable_start_time is None:
                self.altitude_stable_start_time = now

            stable_elapsed = (
                now - self.altitude_stable_start_time
            ).to_sec()

            rospy.loginfo_throttle(
                1.0,
                "altitude stable candidate: root_z=%.3f, target_z=%.3f, vz=%.3f, elapsed=%.2f",
                self.current_root_z,
                self.target_z,
                self.current_vz,
                stable_elapsed
            )

            return stable_elapsed > self.stable_time

        self.altitude_stable_start_time = None
        return False

    # ============================================================
    # IK / trajectory
    # ============================================================

    def compute_aim_yaw(self, target_xyz):
        phi = np.arctan2(
            target_xyz[1] - self.fixed_root_xyz[1],
            target_xyz[0] - self.fixed_root_xyz[0]
        )
        return phi - self.alpha0

    def set_model_state_for_ik(self, yaw):
        q_actual = self.get_actual_q()

        self.fjoint.joint_angle(
            np.r_[self.fixed_root_xyz, 0.0, 0.0, yaw]
        )

        self.robot.joint1.joint_angle(q_actual[0])
        self.robot.joint2.joint_angle(self.args.fix_angle)
        self.robot.joint3.joint_angle(q_actual[2])

    def solve_ik(self, target_xyz):
        result = self.robot.inverse_kinematics(
            Coordinates(pos=target_xyz),
            link_list=self.link_list,
            move_target=self.leg5,
            position_mask="xy",
            rotation_mask=False,
            stop=self.args.ik_stop,
            revert_if_fail=False
        )

        ok = result is not False and result is not None
        return ok

    def compute_command_for_target(self, target_xyz):
        yaw = self.compute_aim_yaw(target_xyz)

        self.set_model_state_for_ik(yaw)
        ok = self.solve_ik(target_xyz)

        q_target = np.array([
            self.robot.joint1.joint_angle(),
            self.robot.joint2.joint_angle(),
            self.robot.joint3.joint_angle()
        ], dtype=float)

        cog_target = self.robot.centroid().copy()

        # 重要：
        # IKで得たCOGのx,yは使うが，zはtarget_zを維持する
        cog_target[2] = self.compute_target_cog_z_from_tf()

        yaw_target = matrix2rpy(self.robot.fc.worldrot())[2]

        leg5_pos = self.leg5.worldpos()
        err_xy = np.linalg.norm(leg5_pos[:2] - target_xyz[:2])

        return {
            "ok": ok,
            "target_xyz": target_xyz,
            "q_target": q_target,
            "cog_target": cog_target,
            "yaw_target": yaw_target,
            "leg5_pos": leg5_pos,
            "err_xy": err_xy,
        }

    def compute_target_cog_z_from_tf(self):
        current_cog_xyz, _ = self.get_xyz_rpy(
        "world",
        self.robot_ns.lstrip("/") + "/cog"
        )

        current_root_xyz, _ = self.get_xyz_rpy(
        "world",
        self.robot_ns.lstrip("/") + "/root"
        )

        root_to_cog_z_offset = current_cog_xyz[2] - current_root_xyz[2]

        target_cog_z = self.target_z + root_to_cog_z_offset
        return target_cog_z

    def compute_circle_target(self, k, center_world):
        theta = 2.0 * np.pi * k / self.args.steps

        return center_world + np.array([
            self.args.radius * np.cos(theta),
            self.args.radius * np.sin(theta),
            0.0
        ])

    def compute_start_center_and_target(self):
        self.fjoint.joint_angle(
            np.r_[self.fixed_root_xyz, 0.0, 0.0, self.root_rpy0[2]]
        )

        self.robot.joint1.joint_angle(self.args.q1_center)
        self.robot.joint2.joint_angle(self.args.fix_angle)
        self.robot.joint3.joint_angle(self.args.q3_center)

        center_world = self.leg5.worldpos().copy()

        start_target = center_world + np.array([
            self.args.radius,
            0.0,
            0.0
        ])

        return center_world, start_target

    # ============================================================
    # publish
    # ============================================================

    def publish_nav_and_joint(self, cog_target, q_target, yaw_target):
        self.q_cmd = self.limit_joint_step(
            self.q_cmd,
            q_target,
            self.args.max_q_step
        )

        self.cog_cmd = self.limit_vector_step(
            self.cog_cmd,
            cog_target,
            self.args.max_cog_step
        )
        self.yaw_cmd = yaw_target

        now = rospy.Time.now()

        nav_msg = FlightNav()
        nav_msg.header.stamp = now
        nav_msg.header.frame_id = "world"
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = float(self.cog_cmd[0])
        nav_msg.target_pos_y = float(self.cog_cmd[1])

        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_z = float(self.cog_cmd[2])

        nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        nav_msg.target_yaw = float(yaw_target)

        self.nav_pub.publish(nav_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = ["joint1", "joint2", "joint3"]
        joint_msg.position = [
            float(self.q_cmd[0]),
            float(self.q_cmd[1]),
            float(self.q_cmd[2])
        ]

        self.joint_pub.publish(joint_msg)

    def publish_debug(self, target_xyz):
        try:
            leg5_xyz, _ = self.get_xyz_rpy(
                "world",
                self.robot_ns.lstrip("/") + "/leg5"
            )
            root_xyz, _ = self.get_xyz_rpy(
                "world",
                self.robot_ns.lstrip("/") + "/root"
            )
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            return

        self.debug_pubs["leg5"].publish(
            self.make_vec3_msg(leg5_xyz)
        )
        self.debug_pubs["target_leg5"].publish(
            self.make_vec3_msg(target_xyz)
        )
        self.debug_pubs["leg5_error"].publish(
            self.make_vec3_msg(leg5_xyz - target_xyz)
        )
        self.debug_pubs["root"].publish(
            self.make_vec3_msg(root_xyz)
        )
        self.debug_pubs["target_root"].publish(
            self.make_vec3_msg(self.fixed_root_xyz)
        )
        self.debug_pubs["root_error"].publish(
            self.make_vec3_msg(root_xyz - self.fixed_root_xyz)
        )

    def make_vec3_msg(self, vec):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.vector.x = float(vec[0])
        msg.vector.y = float(vec[1])
        msg.vector.z = float(vec[2])
        return msg

    # ============================================================
    # viewer
    # ============================================================

    def update_trail_viewer(self, center_world):
        for j, mark in enumerate(self.trail_spheres):
            phi = 2.0 * np.pi * j / self.args.trail_points
            wp = center_world + np.array([
                self.args.radius * np.cos(phi),
                self.args.radius * np.sin(phi),
                0.0
            ])
            mark.newcoords(Coordinates(pos=wp))

    def update_viewer_markers(self, target_xyz):
        if self.target_axis is not None:
            self.target_axis.newcoords(Coordinates(pos=target_xyz))

        if self.root_axis is not None:
            self.root_axis.newcoords(Coordinates(pos=self.fixed_root_xyz))

        if self.viewer is not None:
            self.viewer.redraw()

    # ============================================================
    # state updates
    # ============================================================

    def update_wait_ready(self):
        self.update_root_z_from_tf()

        ready = (
            self.current_q is not None and
            self.current_z is not None and
            self.current_vz is not None and
            self.current_root_z is not None
        )

        if ready:
            rospy.loginfo(
                "ready: q=(%.3f, %.3f, %.3f), z=%.3f, vz=%.3f, root_z=%.3f",
                self.current_q[0],
                self.current_q[1],
                self.current_q[2],
                self.current_z,
                self.current_vz,
                self.current_root_z
            )

            self.q_cmd = self.get_actual_q()
            self.cog_cmd, _ = self.get_xyz_rpy(
                "world",
                self.robot_ns.lstrip("/") + "/cog"
            )

            self.change_state("GO_TARGET_ALTITUDE")
        else:
            rospy.loginfo_throttle(
                2.0,
                "waiting: q=%s, z=%s, vz=%s, root_z=%s",
                str(self.current_q),
                str(self.current_z),
                str(self.current_vz),
                str(self.current_root_z)
            )

    def update_go_target_altitude(self):
        self.update_root_z_from_tf()

        # 高度だけtarget_zへ近づける
        self.publish_z_velocity_command()

        # 現在の関節角を保持
        q_hold = self.q_cmd.copy()
        self.publish_joint_only(q_hold)

        if self.update_altitude_stable_timer():
            self.altitude_stable_start_time = None
            self.cog_cmd, _ = self.get_xyz_rpy(
                "world",
                self.robot_ns.lstrip("/") + "/cog"
            )
            self.change_state("SET_START_POSE")

    def update_set_start_pose(self):
        self.update_root_z_from_tf()

        center_candidate, start_target = self.compute_start_center_and_target()
        command = self.compute_command_for_target(start_target)

        self.publish_nav_and_joint(
            command["cog_target"],
            command["q_target"],
            command["yaw_target"]
        )

        self.publish_debug(start_target)
        self.update_viewer_markers(start_target)

        stable = self.is_start_pose_stable(
            center_candidate,
            start_target
        )

        if stable:
            self.center_world = center_candidate.copy()
            self.update_trail_viewer(self.center_world)

            self.k = 0
            self.errors = []

            self.start_pose_stable_start_time = None

            rospy.loginfo(
                "start pose stabilized. center_world = %s",
                self.center_world
            )

            self.change_state("CIRCLE_TRACKING")

    def update_circle_tracking(self):
        self.update_root_z_from_tf()

        target_xyz = self.compute_circle_target(
            self.k,
            self.center_world
        )

        command = self.compute_command_for_target(target_xyz)

        self.publish_nav_and_joint(
            command["cog_target"],
            command["q_target"],
            command["yaw_target"]
        )

        self.publish_debug(target_xyz)
        self.update_viewer_markers(target_xyz)

        self.errors.append(command["err_xy"])

        rospy.loginfo_throttle(
            1.0,
            "[CIRCLE_TRACKING] k=%d, err_xy=%.4f, root_z=%.3f, target_z=%.3f, q=(%.3f, %.3f, %.3f)",
            self.k,
            command["err_xy"],
            self.current_root_z if self.current_root_z is not None else -999.0,
            self.target_z,
            command["q_target"][0],
            command["q_target"][1],
            command["q_target"][2]
        )

        self.k += 1

        if self.k >= self.args.steps:
            rospy.loginfo(
                "circle tracking completed. mean_err_xy=%.4f",
                np.mean(self.errors)
            )
            self.change_state("HOLD")

    def update_hold(self):
        self.update_root_z_from_tf()
        self.publish_nav_and_joint(
        self.cog_cmd.copy(),
        self.q_cmd.copy(),
        self.yaw_cmd
        )

    # ============================================================
    # stability check
    # ============================================================

    def is_start_pose_stable(self, center_world, start_target):
        now = rospy.Time.now()

        try:
            actual_leg5_xyz, _ = self.get_xyz_rpy(
                "world",
                self.robot_ns.lstrip("/") + "/leg5"
            )
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            return False

        start_err = np.linalg.norm(
            actual_leg5_xyz[:2] - start_target[:2]
        )

        radius_err = (
            np.linalg.norm(actual_leg5_xyz[:2] - center_world[:2])
            - self.args.radius
        )

        altitude_ok = self.is_altitude_stable()

        pose_ok = (
            start_err < self.start_err_tol and
            abs(radius_err) < self.radius_err_tol and
            altitude_ok
        )

        if pose_ok:
            if self.start_pose_stable_start_time is None:
                self.start_pose_stable_start_time = now

            stable_elapsed = (
                now - self.start_pose_stable_start_time
            ).to_sec()

            rospy.loginfo_throttle(
                1.0,
                "[SET_START_POSE] stable candidate: start_err=%.4f, radius_err=%.4f, root_z=%.3f, elapsed=%.2f",
                start_err,
                radius_err,
                self.current_root_z,
                stable_elapsed
            )

            return stable_elapsed > self.start_stable_time

        self.start_pose_stable_start_time = None

        rospy.loginfo_throttle(
            1.0,
            "[SET_START_POSE] start_err=%.4f, radius_err=%.4f, altitude_ok=%s",
            start_err,
            radius_err,
            str(altitude_ok)
        )

        return False

    # ============================================================
    # simple joint hold
    # ============================================================

    def publish_joint_only(self, q):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = [
            float(q[0]),
            float(q[1]),
            float(q[2])
        ]
        self.joint_pub.publish(msg)

    # ============================================================
    # main loop
    # ============================================================

    def spin(self):
        rate = rospy.Rate(self.args.control_hz)

        while not rospy.is_shutdown():
            if self.viewer is not None and not self.viewer.is_active:
                rospy.loginfo("viewer closed. switching to HOLD")
                self.change_state("HOLD")

            if self.state == "WAIT_READY":
                self.update_wait_ready()

            elif self.state == "GO_TARGET_ALTITUDE":
                self.update_go_target_altitude()

            elif self.state == "SET_START_POSE":
                self.update_set_start_pose()

            elif self.state == "CIRCLE_TRACKING":
                self.update_circle_tracking()

            elif self.state == "HOLD":
                self.update_hold()

            rate.sleep()


if __name__ == "__main__":
    node = CeilingCircleIKNode()
    node.spin()
