#!/usr/bin/env python3

import json
import math
import sys
import threading

import numpy as np
import rospy
import tf2_ros
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt8
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix


class CeilingHorizontalTrial:
    HOVER_STATE = 5

    def __init__(self):
        self.robot_ns = self.normalize_ns(rospy.get_param("~robot_ns", "/delta"))
        self.frame_prefix = self.robot_ns.strip("/")

        self.dbar_ref = float(rospy.get_param("~dbar_ref", rospy.get_param("~dbar", 1.5)))
        self.vref = float(rospy.get_param("~vref", 0.2))
        self.q123 = self.parse_vector_param("~q123", [1.4, 1.4, 1.4], 3)
        self.yaw_ref_param = rospy.get_param("~yaw_ref", 0.0)

        self.ceiling_height = float(rospy.get_param("~ceiling_height", 2.73))
        self.rotor_radius = float(rospy.get_param("~rotor_radius", 0.1905))
        self.travel_distance = float(rospy.get_param("~travel_distance", 1.0))
        self.approach_vmax = float(rospy.get_param("~approach_vmax", 0.03))
        self.approach_min_duration = float(rospy.get_param("~approach_min_duration", 3.0))
        self.yaw_align_rate = float(rospy.get_param("~yaw_align_rate", 0.03))
        self.nav_rate = float(rospy.get_param("~nav_rate", 50.0))
        self.settle_duration = float(rospy.get_param("~settle_duration", 1.0))
        self.settle_timeout = float(rospy.get_param("~settle_timeout", 30.0))
        self.hold_duration = float(rospy.get_param("~hold_duration", 1.0))
        self.future_ref_times = self.parse_float_list_param("~future_ref_times", [0.2, 0.4, 0.6, 0.8])

        self.xy_thresh = float(rospy.get_param("~xy_thresh", 0.05))
        self.z_thresh = float(rospy.get_param("~z_thresh", 0.03))
        self.rp_thresh = float(rospy.get_param("~rp_thresh", 0.08))
        self.yaw_thresh = float(rospy.get_param("~yaw_thresh", 0.12))
        self.vel_thresh = float(rospy.get_param("~vel_thresh", 0.08))
        self.dbar_thresh = float(rospy.get_param("~dbar_thresh", 0.08))
        self.joint_thresh = float(rospy.get_param("~joint_thresh", 0.05))
        self.joint_timeout = float(rospy.get_param("~joint_timeout", 15.0))
        self.hover_timeout = float(rospy.get_param("~hover_timeout", 10.0))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 5.0))
        self.require_hover = bool(rospy.get_param("~require_hover", True))

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.cog_frame = rospy.get_param("~cog_frame", self.frame_prefix + "/cog")
        self.rotor1_frame = rospy.get_param("~rotor1_frame", self.frame_prefix + "/thrust1")
        self.ref_frame = rospy.get_param("~ref_frame", self.frame_prefix + "/ceiling_trial_cog_ref")
        self.future_ref_frame_prefix = rospy.get_param("~future_ref_frame_prefix", self.ref_frame + "_plus")

        if self.dbar_ref <= 0.0:
            raise ValueError("~dbar_ref must be positive")
        if self.vref <= 0.0:
            raise ValueError("~vref must be positive")
        if self.rotor_radius <= 0.0:
            raise ValueError("~rotor_radius must be positive")
        if self.approach_vmax <= 0.0:
            raise ValueError("~approach_vmax must be positive")
        if self.yaw_align_rate <= 0.0:
            raise ValueError("~yaw_align_rate must be positive")
        if self.nav_rate <= 0.0:
            raise ValueError("~nav_rate must be positive")

        self.lock = threading.Lock()
        self.reference_lock = threading.Lock()
        self.odom = None
        self.flight_state = None
        self.joint_positions = {}
        self.ref_pose = None
        self.reference_segment = None

        self.nav_pub = rospy.Publisher(self.robot_ns + "/uav/nav", FlightNav, queue_size=1)
        self.joint_pub = rospy.Publisher(self.robot_ns + "/joints_ctrl", JointState, queue_size=1)
        self.reference_pub = rospy.Publisher(self.robot_ns + "/ceiling_horizontal_trial/reference_pose",
                                             PoseStamped, queue_size=1)
        self.phase_pub = rospy.Publisher(self.robot_ns + "/ceiling_horizontal_trial/phase",
                                         String, queue_size=1, latch=True)
        self.condition_pub = rospy.Publisher(self.robot_ns + "/ceiling_horizontal_trial/condition",
                                             String, queue_size=1, latch=True)
        self.error_pub = rospy.Publisher(self.robot_ns + "/ceiling_horizontal_trial/error_xyzrpy",
                                         PoseControlPid, queue_size=1)

        self.odom_sub = rospy.Subscriber(self.robot_ns + "/uav/cog/odom", Odometry, self.odom_callback, queue_size=1)
        self.flight_state_sub = rospy.Subscriber(self.robot_ns + "/flight_state", UInt8,
                                                 self.flight_state_callback, queue_size=1)
        self.joint_state_sub = rospy.Subscriber(self.robot_ns + "/joint_states", JointState,
                                                self.joint_state_callback, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.ref_timer = rospy.Timer(rospy.Duration(0.05), self.publish_reference_timer)

        rospy.on_shutdown(self.stop_request)

    @staticmethod
    def normalize_ns(ns):
        if not ns:
            return ""
        return "/" + ns.strip("/")

    @staticmethod
    def parse_vector_param(name, default, length):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            value = value.strip().strip("[]")
            value = [] if not value else [float(x) for x in value.replace(",", " ").split()]
        value = [float(x) for x in value]
        if len(value) != length:
            raise ValueError("{} must have {} elements".format(name, length))
        return value

    @staticmethod
    def parse_float_list_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            value = value.strip().strip("[]")
            value = [] if not value else [float(x) for x in value.replace(",", " ").split()]
        return [float(x) for x in value]

    @staticmethod
    def shortest_angle_error(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def is_current_yaw_param(value):
        return value is None or (isinstance(value, str) and value.lower() == "current")

    @staticmethod
    def rotate_vector(quaternion_xyzw, vector_xyz):
        matrix = quaternion_matrix(quaternion_xyzw)
        vector = np.array([vector_xyz[0], vector_xyz[1], vector_xyz[2], 0.0])
        return matrix.dot(vector)[:3]

    @staticmethod
    def minimum_jerk(t, duration):
        if duration <= 0.0:
            return 1.0, 0.0
        tau = max(0.0, min(1.0, t / duration))
        s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        ds_dt = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
        return s, ds_dt

    def odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def flight_state_callback(self, msg):
        with self.lock:
            self.flight_state = msg.data

    def joint_state_callback(self, msg):
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                self.joint_positions[name] = position

    def get_odom(self):
        with self.lock:
            return self.odom

    def get_flight_state(self):
        with self.lock:
            return self.flight_state

    def get_joint_positions(self):
        with self.lock:
            return dict(self.joint_positions)

    def get_position_rpy_vel(self):
        odom = self.get_odom()
        if odom is None:
            return None
        pos = odom.pose.pose.position
        quat_msg = odom.pose.pose.orientation
        quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        rpy = euler_from_quaternion(quat)
        vel = odom.twist.twist.linear
        return np.array([pos.x, pos.y, pos.z]), np.array(rpy), np.array([vel.x, vel.y, vel.z]), quat

    def wait_for_initial_data(self):
        start = rospy.Time.now()
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            has_odom = self.get_odom() is not None
            has_state = self.get_flight_state() is not None
            if has_odom and (has_state or not self.require_hover):
                break
            if (rospy.Time.now() - start).to_sec() > self.hover_timeout:
                raise RuntimeError("timeout while waiting for odom/flight_state")
            rospy.loginfo_throttle(1.0, "waiting for %s/uav/cog/odom and flight_state", self.robot_ns)
            rate.sleep()

        if not self.require_hover:
            return

        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.get_flight_state() == self.HOVER_STATE:
                return
            if (rospy.Time.now() - start).to_sec() > self.hover_timeout:
                raise RuntimeError("flight_state is not HOVER_STATE")
            rospy.loginfo_throttle(1.0, "waiting for HOVER_STATE, current=%s", str(self.get_flight_state()))
            rate.sleep()

    def publish_phase(self, phase):
        self.phase_pub.publish(String(data=phase))
        rospy.loginfo("[ceiling_trial] phase: %s", phase)

    def publish_condition(self, x_ref, y_ref, z_ref, yaw_ref):
        condition = {
            "dbar_ref_rotor1": self.dbar_ref,
            "vref": self.vref,
            "q123": self.q123,
            "x_ref": x_ref,
            "y_ref": y_ref,
            "z_ref": z_ref,
            "roll_ref": 0.0,
            "pitch_ref": 0.0,
            "yaw_ref": yaw_ref,
            "ceiling_height": self.ceiling_height,
            "rotor_radius": self.rotor_radius,
            "travel_distance": self.travel_distance,
            "cog_ref_frame": self.ref_frame,
            "future_ref_times": self.future_ref_times,
            "future_ref_frame_prefix": self.future_ref_frame_prefix,
        }
        self.condition_pub.publish(String(data=json.dumps(condition, sort_keys=True)))

    def set_reference_pose(self, x, y, z, yaw):
        with self.reference_lock:
            self.ref_pose = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw": float(yaw),
            }

    def set_reference_segment(self, start_time, start_pos, goal_pos, duration, start_yaw, goal_yaw=None):
        if goal_yaw is None:
            goal_yaw = start_yaw
        with self.reference_lock:
            self.reference_segment = {
                "start_time": start_time,
                "start_pos": np.array(start_pos, dtype=float),
                "goal_pos": np.array(goal_pos, dtype=float),
                "duration": float(duration),
                "start_yaw": float(start_yaw),
                "goal_yaw": float(goal_yaw),
            }

    def set_static_reference_segment(self, pos, yaw):
        self.set_reference_segment(rospy.Time.now(), pos, pos, 0.0, yaw)

    def future_ref_frame_name(self, offset):
        label = "{:.1f}".format(offset).replace(".", "p")
        return "{}_{}s".format(self.future_ref_frame_prefix, label)

    def reference_pose_at(self, offset):
        with self.reference_lock:
            ref_pose = None if self.ref_pose is None else dict(self.ref_pose)
            segment = self.reference_segment
            if segment is not None:
                segment = {
                    "start_time": segment["start_time"],
                    "start_pos": np.array(segment["start_pos"], dtype=float),
                    "goal_pos": np.array(segment["goal_pos"], dtype=float),
                    "duration": segment["duration"],
                    "start_yaw": segment["start_yaw"],
                    "goal_yaw": segment["goal_yaw"],
                }

        if segment is None:
            return ref_pose

        elapsed = (rospy.Time.now() - segment["start_time"]).to_sec() + offset
        s, _ds_dt = self.minimum_jerk(elapsed, segment["duration"])
        pos = segment["start_pos"] + (segment["goal_pos"] - segment["start_pos"]) * s
        yaw_delta = self.shortest_angle_error(segment["goal_yaw"], segment["start_yaw"])
        yaw = self.normalize_angle(segment["start_yaw"] + yaw_delta * s)
        return {
            "x": float(pos[0]),
            "y": float(pos[1]),
            "z": float(pos[2]),
            "yaw": yaw,
        }

    def publish_reference_timer(self, _event):
        if self.reference_pose_at(0.0) is None:
            return
        self.publish_reference()

    def publish_reference(self):
        current_pose = self.reference_pose_at(0.0)
        if current_pose is None:
            return

        stamp = rospy.Time.now()
        self.publish_reference_transform(stamp, current_pose, self.ref_frame)

        for offset in self.future_ref_times:
            future_pose = self.reference_pose_at(offset)
            if future_pose is not None:
                self.publish_reference_transform(stamp, future_pose, self.future_ref_frame_name(offset))

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        quat = quaternion_from_euler(0.0, 0.0, current_pose["yaw"])
        pose.pose.position.x = current_pose["x"]
        pose.pose.position.y = current_pose["y"]
        pose.pose.position.z = current_pose["z"]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.reference_pub.publish(pose)

    def publish_reference_transform(self, stamp, pose, child_frame):
        quat = quaternion_from_euler(0.0, 0.0, pose["yaw"])

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = pose["x"]
        transform.transform.translation.y = pose["y"]
        transform.transform.translation.z = pose["z"]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(transform)

    def publish_joint_command(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = self.q123
        self.joint_pub.publish(msg)

    def set_q123_and_wait(self):
        self.publish_phase("set_q123")
        start = rospy.Time.now()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.publish_joint_command()
            if self.check_joint_convergence():
                rospy.loginfo("[ceiling_trial] q123 converged")
                return
            if (rospy.Time.now() - start).to_sec() > self.joint_timeout:
                raise RuntimeError("q123 convergence timeout")
            rate.sleep()

    def check_joint_convergence(self):
        positions = self.get_joint_positions()
        errors = []
        for name, target in zip(["joint1", "joint2", "joint3"], self.q123):
            if name not in positions:
                rospy.loginfo_throttle(1.0, "waiting for %s in joint_states", name)
                return False
            errors.append(target - positions[name])
        rospy.loginfo_throttle(1.0, "q123 error: %s", ["{:.3f}".format(e) for e in errors])
        return all(abs(error) < self.joint_thresh for error in errors)

    def lookup_rotor1_offset_from_cog(self):
        start = rospy.Time.now()
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform(self.cog_frame, self.rotor1_frame,
                                                        rospy.Time(0), rospy.Duration(0.2))
                t = trans.transform.translation
                return np.array([t.x, t.y, t.z])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                if (rospy.Time.now() - start).to_sec() > self.tf_timeout:
                    raise RuntimeError("failed to lookup {} -> {}: {}".format(
                        self.cog_frame, self.rotor1_frame, exc))
                rospy.loginfo_throttle(1.0, "waiting for TF %s -> %s", self.cog_frame, self.rotor1_frame)
                rate.sleep()
        raise RuntimeError("shutdown while waiting for rotor1 TF")

    def compute_z_ref(self, rotor1_offset_cog, yaw_ref):
        quat_ref = quaternion_from_euler(0.0, 0.0, yaw_ref)
        offset_world = self.rotate_vector(quat_ref, rotor1_offset_cog)
        rotor1_distance = self.dbar_ref * self.rotor_radius
        return self.ceiling_height - rotor1_distance - offset_world[2]

    def current_dbar1(self, rotor1_offset_cog):
        data = self.get_position_rpy_vel()
        if data is None:
            return None
        pos, _rpy, _vel, quat = data
        offset_world = self.rotate_vector(quat, rotor1_offset_cog)
        rotor1_z = pos[2] + offset_world[2]
        return (self.ceiling_height - rotor1_z) / self.rotor_radius

    def publish_nav(self, pos, vel, yaw_ref, yaw_rate=0.0):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        msg.pos_z_nav_mode = FlightNav.POS_VEL_MODE
        msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x = float(pos[0])
        msg.target_pos_y = float(pos[1])
        msg.target_pos_z = float(pos[2])
        msg.target_vel_x = float(vel[0])
        msg.target_vel_y = float(vel[1])
        msg.target_vel_z = float(vel[2])
        msg.target_yaw = float(yaw_ref)
        msg.target_omega_z = float(yaw_rate)
        self.nav_pub.publish(msg)
        self.set_reference_pose(pos[0], pos[1], pos[2], yaw_ref)
        self.publish_reference()

    def publish_error_xyzrpy(self, target_pos, target_vel, yaw_ref):
        data = self.get_position_rpy_vel()
        if data is None:
            return

        pos, rpy, vel, _quat = data
        target_pos = np.array(target_pos, dtype=float)
        target_vel = np.array(target_vel, dtype=float)

        msg = PoseControlPid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame

        msg.x.target_p = float(target_pos[0])
        msg.x.err_p = float(target_pos[0] - pos[0])
        msg.x.target_d = float(target_vel[0])
        msg.x.err_d = float(target_vel[0] - vel[0])

        msg.y.target_p = float(target_pos[1])
        msg.y.err_p = float(target_pos[1] - pos[1])
        msg.y.target_d = float(target_vel[1])
        msg.y.err_d = float(target_vel[1] - vel[1])

        msg.z.target_p = float(target_pos[2])
        msg.z.err_p = float(target_pos[2] - pos[2])
        msg.z.target_d = float(target_vel[2])
        msg.z.err_d = float(target_vel[2] - vel[2])

        msg.roll.target_p = 0.0
        msg.roll.err_p = float(0.0 - rpy[0])
        msg.pitch.target_p = 0.0
        msg.pitch.err_p = float(0.0 - rpy[1])
        msg.yaw.target_p = float(yaw_ref)
        msg.yaw.err_p = float(self.shortest_angle_error(yaw_ref, rpy[2]))

        self.error_pub.publish(msg)

    def execute_minimum_jerk(self, start_pos, goal_pos, duration, yaw_ref, phase):
        self.publish_phase(phase)
        start_time = rospy.Time.now()
        rate = rospy.Rate(self.nav_rate)
        start_pos = np.array(start_pos, dtype=float)
        goal_pos = np.array(goal_pos, dtype=float)
        delta = goal_pos - start_pos
        self.set_reference_segment(start_time, start_pos, goal_pos, duration, yaw_ref)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            s, ds_dt = self.minimum_jerk(elapsed, duration)
            pos = start_pos + delta * s
            vel = delta * ds_dt
            self.publish_nav(pos, vel, yaw_ref)
            if phase == "horizontal_x":
                self.publish_error_xyzrpy(pos, vel, yaw_ref)
            if elapsed >= duration:
                break
            rate.sleep()

        self.publish_nav(goal_pos, np.zeros(3), yaw_ref)
        self.set_static_reference_segment(goal_pos, yaw_ref)

    def execute_yaw_align(self, target_pos, start_yaw, goal_yaw):
        self.publish_phase("yaw_align")
        start_time = rospy.Time.now()
        rate = rospy.Rate(self.nav_rate)
        target_pos = np.array(target_pos, dtype=float)
        yaw_delta = self.shortest_angle_error(goal_yaw, start_yaw)
        duration = 1.875 * abs(yaw_delta) / self.yaw_align_rate
        rospy.loginfo("[ceiling_trial] yaw align delta=%.3f rad, duration=%.3f sec",
                      yaw_delta, duration)
        self.set_reference_segment(start_time, target_pos, target_pos,
                                   duration, start_yaw, goal_yaw)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            s, ds_dt = self.minimum_jerk(elapsed, duration)
            yaw = self.normalize_angle(start_yaw + yaw_delta * s)
            yaw_rate = yaw_delta * ds_dt
            self.publish_nav(target_pos, np.zeros(3), yaw, yaw_rate)
            if elapsed >= duration:
                break
            rate.sleep()

        self.publish_nav(target_pos, np.zeros(3), goal_yaw)
        self.set_static_reference_segment(target_pos, goal_yaw)

    def wait_until_yaw_stable(self, target_pos, yaw_ref):
        self.publish_phase("yaw_settle")
        rate = rospy.Rate(self.nav_rate)
        settle_start = rospy.Time.now()
        stable_start = None
        target_pos = np.array(target_pos, dtype=float)
        self.set_static_reference_segment(target_pos, yaw_ref)

        while not rospy.is_shutdown():
            if (rospy.Time.now() - settle_start).to_sec() > self.settle_timeout:
                raise RuntimeError("yaw settle timeout")

            self.publish_nav(target_pos, np.zeros(3), yaw_ref)
            data = self.get_position_rpy_vel()
            if data is None:
                rate.sleep()
                continue

            pos, rpy, vel, _quat = data
            pos_error = pos - target_pos
            yaw_error = self.shortest_angle_error(yaw_ref, rpy[2])
            stable = (
                abs(pos_error[0]) < self.xy_thresh and
                abs(pos_error[1]) < self.xy_thresh and
                abs(pos_error[2]) < self.z_thresh and
                abs(rpy[0]) < self.rp_thresh and
                abs(rpy[1]) < self.rp_thresh and
                abs(yaw_error) < self.yaw_thresh and
                np.linalg.norm(vel) < self.vel_thresh
            )

            rospy.loginfo_throttle(
                1.0,
                "yaw settle err xyz=[%.3f %.3f %.3f], rp=[%.3f %.3f], yaw=%.3f",
                pos_error[0], pos_error[1], pos_error[2], rpy[0], rpy[1], yaw_error)

            now = rospy.Time.now()
            if stable:
                if stable_start is None:
                    stable_start = now
                if (now - stable_start).to_sec() >= self.settle_duration:
                    rospy.loginfo("[ceiling_trial] yaw stable for %.2f sec", self.settle_duration)
                    return
            else:
                stable_start = None
            rate.sleep()

        raise RuntimeError("shutdown while waiting for yaw stable state")

    def wait_until_stable(self, target_pos, yaw_ref, rotor1_offset_cog):
        self.publish_phase("settle")
        rate = rospy.Rate(self.nav_rate)
        settle_start = rospy.Time.now()
        stable_start = None
        target_pos = np.array(target_pos, dtype=float)
        self.set_static_reference_segment(target_pos, yaw_ref)

        while not rospy.is_shutdown():
            if (rospy.Time.now() - settle_start).to_sec() > self.settle_timeout:
                raise RuntimeError("settle timeout")

            self.publish_nav(target_pos, np.zeros(3), yaw_ref)
            data = self.get_position_rpy_vel()
            dbar1 = self.current_dbar1(rotor1_offset_cog)
            if data is None or dbar1 is None:
                rate.sleep()
                continue

            pos, rpy, vel, _quat = data
            pos_error = pos - target_pos
            yaw_error = self.shortest_angle_error(yaw_ref, rpy[2])
            joint_ok = self.check_joint_convergence()

            stable = (
                abs(pos_error[0]) < self.xy_thresh and
                abs(pos_error[1]) < self.xy_thresh and
                abs(pos_error[2]) < self.z_thresh and
                abs(rpy[0]) < self.rp_thresh and
                abs(rpy[1]) < self.rp_thresh and
                abs(yaw_error) < self.yaw_thresh and
                np.linalg.norm(vel) < self.vel_thresh and
                abs(dbar1 - self.dbar_ref) < self.dbar_thresh and
                joint_ok
            )

            rospy.loginfo_throttle(
                1.0,
                "settle err xyz=[%.3f %.3f %.3f], rp=[%.3f %.3f], yaw=%.3f, dbar1=%.3f",
                pos_error[0], pos_error[1], pos_error[2], rpy[0], rpy[1], yaw_error, dbar1)

            now = rospy.Time.now()
            if stable:
                if stable_start is None:
                    stable_start = now
                if (now - stable_start).to_sec() >= self.settle_duration:
                    rospy.loginfo("[ceiling_trial] stable for %.2f sec", self.settle_duration)
                    return
            else:
                stable_start = None
            rate.sleep()

        raise RuntimeError("shutdown while waiting for stable state")

    def stop_request(self):
        data = self.get_position_rpy_vel()
        if data is None:
            return
        pos, rpy, _vel, _quat = data
        yaw = rpy[2]
        self.set_static_reference_segment(pos, yaw)
        self.publish_nav(pos, np.zeros(3), yaw)

    def run(self):
        self.wait_for_initial_data()

        data = self.get_position_rpy_vel()
        if data is None:
            raise RuntimeError("missing odom")
        initial_pos, initial_rpy, _vel, _quat = data
        x_ref = initial_pos[0]
        y_ref = initial_pos[1]
        yaw_ref = initial_rpy[2] if self.is_current_yaw_param(self.yaw_ref_param) else float(self.yaw_ref_param)
        yaw_ref = self.normalize_angle(yaw_ref)

        rospy.loginfo("[ceiling_trial] start xy=(%.3f, %.3f), yaw_ref=%.3f", x_ref, y_ref, yaw_ref)

        self.set_q123_and_wait()
        rotor1_offset_cog = self.lookup_rotor1_offset_from_cog()

        data = self.get_position_rpy_vel()
        if data is None:
            raise RuntimeError("missing odom before yaw align")
        yaw_align_pos, yaw_align_rpy, _vel, _quat = data
        self.execute_yaw_align(yaw_align_pos, yaw_align_rpy[2], yaw_ref)
        self.wait_until_yaw_stable(yaw_align_pos, yaw_ref)

        data = self.get_position_rpy_vel()
        if data is None:
            raise RuntimeError("missing odom after yaw align")
        aligned_pos, _aligned_rpy, _vel, _quat = data
        x_ref = aligned_pos[0]
        y_ref = aligned_pos[1]

        z_ref = self.compute_z_ref(rotor1_offset_cog, yaw_ref)
        target_pos = np.array([x_ref, y_ref, z_ref])

        if z_ref >= self.ceiling_height:
            raise RuntimeError("computed z_ref is above ceiling_height")

        self.set_reference_pose(x_ref, y_ref, z_ref, yaw_ref)
        self.set_static_reference_segment(target_pos, yaw_ref)
        self.publish_condition(x_ref, y_ref, z_ref, yaw_ref)

        current_pos = self.get_position_rpy_vel()[0]
        approach_distance = np.linalg.norm(target_pos - current_pos)
        approach_duration = max(self.approach_min_duration, approach_distance / self.approach_vmax)
        self.execute_minimum_jerk(current_pos, target_pos, approach_duration, yaw_ref, "approach_height")

        self.wait_until_stable(target_pos, yaw_ref, rotor1_offset_cog)

        goal_pos = np.array([x_ref + self.travel_distance, y_ref, z_ref])
        horizontal_duration = 1.875 * abs(self.travel_distance) / self.vref
        self.execute_minimum_jerk(target_pos, goal_pos, horizontal_duration, yaw_ref, "horizontal_x")

        self.publish_phase("hold_final")
        self.set_static_reference_segment(goal_pos, yaw_ref)
        hold_start = rospy.Time.now()
        rate = rospy.Rate(self.nav_rate)
        while not rospy.is_shutdown() and (rospy.Time.now() - hold_start).to_sec() < self.hold_duration:
            self.publish_nav(goal_pos, np.zeros(3), yaw_ref)
            rate.sleep()

        self.publish_phase("done")
        rospy.loginfo("[ceiling_trial] completed")


def main():
    rospy.init_node("ceiling_horizontal_trial")
    try:
        CeilingHorizontalTrial().run()
    except Exception as exc:
        rospy.logerr("[ceiling_trial] %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
