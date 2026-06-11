#!/usr/bin/env python
from __future__ import print_function

import math

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav


class CeilingEffectRunNode(object):
    def __init__(self):
        rospy.init_node("ceiling_effect_run_node")

        self.robot_ns = rospy.get_param("~robot_ns", "/hydrus")
        if not self.robot_ns.startswith("/"):
            self.robot_ns = "/" + self.robot_ns
        self.robot_ns = self.robot_ns.rstrip("/")

        # ===== 高度制御パラメータ =====
        self.target_d_R = rospy.get_param("~target_d_R", rospy.get_param(self.robot_ns + "/target_d_R", 8.0))
        self.ceiling_height = rospy.get_param(self.robot_ns + "/ceiling_height")
        self.ceiling_distance_offset = rospy.get_param(self.robot_ns + "/ceiling_distance_offset")
        self.rotor_radius = rospy.get_param(self.robot_ns + "/rotor_radius")

        self.target_d = self.target_d_R * self.rotor_radius
        self.target_z = (
            self.ceiling_height
            - self.ceiling_distance_offset
            - self.target_d
        )

        # 高度安定判定
        self.z_threshold = rospy.get_param("~z_threshold", 0.10)
        self.vz_threshold = rospy.get_param("~vz_threshold", 0.08)
        self.stable_time = rospy.get_param("~stable_time", 1.5)

        # 目標高度へ移動するときの速度制御
        self.kp_z = rospy.get_param("~kp_z", 0.10)
        self.max_vz = rospy.get_param("~max_vz", 0.05)

        # ===== 関節角度パラメータ =====
        self.joint_limit_margin = rospy.get_param("~joint_limit_margin", 0.02)
        self.joint_min = -math.pi / 2.0 + self.joint_limit_margin
        self.joint_max =  math.pi / 2.0 - self.joint_limit_margin

        self.motion_period = rospy.get_param("~motion_period", 120.0)
        self.motion_cycles = rospy.get_param("~motion_cycles", 1.0)
        self.motion_duration = rospy.get_param(
            "~motion_duration",
            self.motion_period * self.motion_cycles
        )

        # ===== joint ごとの time waypoint =====
        # q1:
        #   前回と同じ。2個目の waypoint は使わず，1個目 -> 3個目を直接接続。
        self.q1_times = rospy.get_param(
            "~q1_times",
            [0.0, 27.0, 42.0, 70.5, 93.0, 120.0]
        )
        self.q1_points = rospy.get_param(
            "~q1_points",
            [1.40, 0.50, 0.50, 1.40, 0.70, 1.40]
        )

        # q2:
        #   40秒付近の waypoint q2=0.90 は通過しない。
        #   その前後の点，つまり 27秒の q2=0.50 と 60秒の q2=1.40 を
        #   3次 smoothstep で直接きれいに接続する。
        self.q2_times = rospy.get_param(
            "~q2_times",
            [0.0, 15.0, 27.0, 60.0, 120.0]
        )
        self.q2_points = rospy.get_param(
            "~q2_points",
            [0.50, 0.50, 0.50, 1.40, 0.50]
        )

        # q3:
        #   40秒付近の phase q3=0.90 をしばらく維持してから，
        #   70秒付近の点 q3=0.50 へ移動する。
        #   デフォルトでは 42秒 -> 55秒 で q3=0.90 を保持し，
        #   55秒 -> 70.5秒 で q3=0.50 へ移動。
        self.q3_times = rospy.get_param(
            "~q3_times",
            [0.0, 15.0, 27.0, 42.0, 55.0, 70.5, 93.0, 120.0]
        )
        self.q3_points = rospy.get_param(
            "~q3_points",
            [1.40, 1.40, 1.40, 0.90, 0.90, 0.50, 0.50, 1.40]
        )

        self.validate_joint_times_and_points()

        self.q1_start = self.clamp(self.q1_points[0], self.joint_min, self.joint_max)
        self.q2_start = self.clamp(self.q2_points[0], self.joint_min, self.joint_max)
        self.q3_start = self.clamp(self.q3_points[0], self.joint_min, self.joint_max)

        self.q1_final = self.q1_start
        self.q2_final = self.q2_start
        self.q3_final = self.q3_start

        # 初期姿勢へ移動するときの関節速度
        self.joint_set_speed = rospy.get_param("~joint_set_speed", 0.03)
        self.min_joint_set_time = rospy.get_param("~min_joint_set_time", 1.0)

        # ===== 現在値 =====
        self.current_cog_x = None
        self.current_cog_y = None
        self.current_z = None
        self.current_vz = None
        self.current_root_z = None

        self.current_q1 = None
        self.current_q2 = None
        self.current_q3 = None

        self.current_cog_yaw = None

        # ===== COG保持目標値 =====
        self.hold_cog_x = None
        self.hold_cog_y = None
        self.hold_cog_yaw = None

        # ===== 初期姿勢移動用 =====
        self.q1_init = None
        self.q2_init = None
        self.q3_init = None
        self.start_joint_set_time = None
        self.start_joint_motion_start_time = None

        # ===== 状態管理 =====
        self.state = "WAIT_JOINT_STATE_AND_ODOM"
        self.state_start_time = rospy.Time.now()
        self.stable_start_time = None
        self.motion_start_time = None
        self.tf_listener = tf.TransformListener()

        # ===== Subscriber =====
        self.joint_state_sub = rospy.Subscriber(
            self.robot_ns + "/joint_states",
            JointState,
            self.joint_state_callback
        )

        self.odom_sub = rospy.Subscriber(
            self.robot_ns + "/uav/baselink/odom",
            Odometry,
            self.odom_callback
        )

        self.cog_odom_sub = rospy.Subscriber(
            self.robot_ns + "/uav/cog/odom",
            Odometry,
            self.cog_odom_callback
        )

        # ===== Publisher =====
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

        self.timer = rospy.Timer(rospy.Duration(0.02), self.update)

        rospy.loginfo("ceiling_effect_run_node started")
        rospy.loginfo("robot_ns = %s", self.robot_ns)
        rospy.loginfo("target_d_R = %.3f", self.target_d_R)
        rospy.loginfo("target_z = %.3f [m]", self.target_z)
        rospy.loginfo("joint trajectory: joint-specific timed cubic smoothstep")
        rospy.loginfo("motion_period = %.3f [s]", self.motion_period)
        rospy.loginfo("motion_cycles = %.3f", self.motion_cycles)
        rospy.loginfo("motion_duration = %.3f [s]", self.motion_duration)
        rospy.loginfo("q1_times = %s", str(self.q1_times))
        rospy.loginfo("q2_times = %s", str(self.q2_times))
        rospy.loginfo("q3_times = %s", str(self.q3_times))
        rospy.loginfo("start pose: q1=%.3f, q2=%.3f, q3=%.3f", self.q1_start, self.q2_start, self.q3_start)
        rospy.loginfo("joint_set_speed = %.3f [rad/s]", self.joint_set_speed)

    def validate_joint_times_and_points(self):
        for name, times, points in [
            ("q1", self.q1_times, self.q1_points),
            ("q2", self.q2_times, self.q2_points),
            ("q3", self.q3_times, self.q3_points),
        ]:
            if len(times) != len(points):
                raise rospy.ROSException("%s_times and %s_points must have the same length" % (name, name))
            if len(times) < 2:
                raise rospy.ROSException("%s requires at least 2 points" % name)
            if abs(times[0]) > 1.0e-6:
                raise rospy.ROSException("%s_times[0] must be 0.0" % name)
            for i in range(len(times) - 1):
                if times[i + 1] <= times[i]:
                    raise rospy.ROSException("%s_times must be strictly increasing" % name)
            if abs(times[-1] - self.motion_period) > 1.0e-6:
                raise rospy.ROSException("%s_times[-1] must be equal to motion_period" % name)

        # 周期軌道なので，最後の値は最初の値にそろえる。
        for name, points in [
            ("q1", self.q1_points),
            ("q2", self.q2_points),
            ("q3", self.q3_points),
        ]:
            if abs(points[0] - points[-1]) > 1.0e-6:
                rospy.logwarn("%s is not periodic. Set last point equal to first point.", name)
                points[-1] = points[0]

        self.q1_points = [self.clamp(q, self.joint_min, self.joint_max) for q in self.q1_points]
        self.q2_points = [self.clamp(q, self.joint_min, self.joint_max) for q in self.q2_points]
        self.q3_points = [self.clamp(q, self.joint_min, self.joint_max) for q in self.q3_points]

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        if "joint1" in name_to_pos:
            self.current_q1 = name_to_pos["joint1"]
        if "joint2" in name_to_pos:
            self.current_q2 = name_to_pos["joint2"]
        if "joint3" in name_to_pos:
            self.current_q3 = name_to_pos["joint3"]

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z
        self.current_vz = msg.twist.twist.linear.z

    def cog_odom_callback(self, msg):
        self.current_cog_x = msg.pose.pose.position.x
        self.current_cog_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        self.current_cog_yaw = yaw

    def update_root_z_from_tf(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(
                "world",
                self.robot_ns.lstrip("/") + "/root",
                rospy.Time(0)
            )
            self.current_root_z = trans[2]
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF: world -> %s/root",
                self.robot_ns.lstrip("/")
            )
            return False

    def change_state(self, next_state):
        rospy.loginfo("state: %s -> %s", self.state, next_state)
        self.state = next_state
        self.state_start_time = rospy.Time.now()

    def clamp(self, x, min_value, max_value):
        return max(min(x, max_value), min_value)

    def smooth_step(self, t, T):
        if T <= 0.0:
            return 1.0

        s = t / T
        s = self.clamp(s, 0.0, 1.0)

        return 3.0 * s * s - 2.0 * s * s * s

    def smoothstep01(self, s):
        s = self.clamp(s, 0.0, 1.0)
        return 3.0 * s * s - 2.0 * s * s * s

    def setup_start_joint_motion(self):
        self.q1_init = self.current_q1
        self.q2_init = self.current_q2
        self.q3_init = self.current_q3

        if self.joint_set_speed <= 0.0:
            rospy.logwarn("joint_set_speed <= 0. Use default 0.03 rad/s")
            self.joint_set_speed = 0.03

        max_diff = max(
            abs(self.q1_start - self.q1_init),
            abs(self.q2_start - self.q2_init),
            abs(self.q3_start - self.q3_init)
        )

        self.start_joint_set_time = max(
            max_diff / self.joint_set_speed,
            self.min_joint_set_time
        )

        self.start_joint_motion_start_time = rospy.Time.now()

        rospy.loginfo(
            "Setting start joints: "
            "q1 %.3f -> %.3f, q2 %.3f -> %.3f, q3 %.3f -> %.3f, duration %.3f [s]",
            self.q1_init,
            self.q1_start,
            self.q2_init,
            self.q2_start,
            self.q3_init,
            self.q3_start,
            self.start_joint_set_time
        )

    def start_joint_trajectory(self, t):
        """
        最初の姿勢合わせ:
        現在の q1, q2, q3 から
        trajectory の開始姿勢 q1_start, q2_start, q3_start へ移動。
        """
        s = self.smooth_step(t, self.start_joint_set_time)

        q1 = self.q1_init + s * (self.q1_start - self.q1_init)
        q2 = self.q2_init + s * (self.q2_start - self.q2_init)
        q3 = self.q3_init + s * (self.q3_start - self.q3_init)

        return q1, q2, q3

    def publish_z_velocity_command(self):
        if (
            self.current_root_z is None or
            self.hold_cog_x is None or
            self.hold_cog_y is None or
            self.hold_cog_yaw is None
        ):
            return

        error_z = self.target_z - self.current_root_z
        vz_cmd = self.kp_z * error_z
        vz_cmd = self.clamp(vz_cmd, -self.max_vz, self.max_vz)

        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG
        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        nav_msg.target_vel_z = vz_cmd

        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = self.hold_cog_x
        nav_msg.target_pos_y = self.hold_cog_y

        nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        nav_msg.target_yaw = self.hold_cog_yaw

        self.nav_pub.publish(nav_msg)

        rospy.loginfo_throttle(
            1.0,
            "[GO_TARGET_ALTITUDE] hold_x=%.3f, hold_y=%.3f, hold_yaw=%.3f, root_z=%.3f, target_z=%.3f, vz_cmd=%.3f",
            self.hold_cog_x,
            self.hold_cog_y,
            self.hold_cog_yaw,
            self.current_root_z,
            self.target_z,
            vz_cmd
        )

    def publish_cog_xyz_yaw_position_command(self):
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = self.hold_cog_x
        nav_msg.target_pos_y = self.hold_cog_y

        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_z = self.target_z

        nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        nav_msg.target_yaw = self.hold_cog_yaw

        self.nav_pub.publish(nav_msg)

    def publish_joint_command(self, q1, q2, q3):
        q1 = self.clamp(q1, self.joint_min, self.joint_max)
        q2 = self.clamp(q2, self.joint_min, self.joint_max)
        q3 = self.clamp(q3, self.joint_min, self.joint_max)

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = [q1, q2, q3]

        # velocity は「この速度で動け」という意味ではなく、
        # JointState の velocity 欄として joint_set_speed を入れておく。
        msg.velocity = [self.joint_set_speed, self.joint_set_speed, self.joint_set_speed]

        msg.effort = []

        self.joint_pub.publish(msg)

        rospy.loginfo_throttle(
            2.0,
            "[State: %s] joints_ctrl: q1=%.3f, q2=%.3f, q3=%.3f",
            self.state,
            q1,
            q2,
            q3
        )

    def is_altitude_stable(self):
        if self.current_z is None or self.current_vz is None:
            return False

        z_error = abs(self.target_z - self.current_z)
        vz_abs = abs(self.current_vz)

        return z_error < self.z_threshold and vz_abs < self.vz_threshold

    def timed_smoothstep(self, t, times, values):
        """
        joint ごとの非等間隔 time waypoint に対する 3次 smoothstep 補間。
        同じ値を連続して置いた区間は hold になる。
        """
        if t <= times[0]:
            return self.clamp(values[0], self.joint_min, self.joint_max)
        if t >= times[-1]:
            return self.clamp(values[-1], self.joint_min, self.joint_max)

        seg = 0
        for i in range(len(times) - 1):
            if times[i] <= t <= times[i + 1]:
                seg = i
                break

        t0 = times[seg]
        t1 = times[seg + 1]
        q0 = values[seg]
        q1 = values[seg + 1]

        dt = t1 - t0
        if dt <= 0.0:
            return self.clamp(q0, self.joint_min, self.joint_max)

        u = (t - t0) / dt
        h = self.smoothstep01(u)

        q = q0 + h * (q1 - q0)
        return self.clamp(q, self.joint_min, self.joint_max)

    def periodic_joint_trajectory(self, t):
        if self.motion_period <= 0.0:
            rospy.logwarn_throttle(2.0, "motion_period <= 0. Use default 120 s")
            self.motion_period = 120.0

        tp = t % self.motion_period

        q1 = self.timed_smoothstep(tp, self.q1_times, self.q1_points)
        q2 = self.timed_smoothstep(tp, self.q2_times, self.q2_points)
        q3 = self.timed_smoothstep(tp, self.q3_times, self.q3_points)

        return q1, q2, q3

    def slow_joint_trajectory(self, t):
        """
        メイン実験フェーズ:
        高度制御，x, y, yaw の指示はそのまま。
        q1, q2, q3 のみ joint-specific time waypoint で別々に動かす。
        """
        return self.periodic_joint_trajectory(t)

    def update(self, event):
        now = rospy.Time.now()
        self.update_root_z_from_tf()

        if self.state == "WAIT_JOINT_STATE_AND_ODOM":
            if (
                self.current_q1 is not None and
                self.current_q2 is not None and
                self.current_q3 is not None and
                self.current_z is not None and
                self.current_vz is not None and
                self.current_root_z is not None
            ):
                rospy.loginfo(
                    "current state received: q1=%.3f, q2=%.3f, q3=%.3f, z=%.3f, vz=%.3f, root_z=%.3f",
                    self.current_q1,
                    self.current_q2,
                    self.current_q3,
                    self.current_z,
                    self.current_vz,
                    self.current_root_z
                )

                self.setup_start_joint_motion()
                self.change_state("SET_START_JOINTS")
            else:
                rospy.loginfo_throttle(
                    2.0,
                    "waiting for joint_states and odom: q1=%s, q2=%s, q3=%s, z=%s, vz=%s, root_z=%s",
                    str(self.current_q1),
                    str(self.current_q2),
                    str(self.current_q3),
                    str(self.current_z),
                    str(self.current_vz),
                    str(self.current_root_z)
                )

        elif self.state == "SET_START_JOINTS":
            t = (now - self.start_joint_motion_start_time).to_sec()
            q1, q2, q3 = self.start_joint_trajectory(t)

            self.publish_joint_command(q1, q2, q3)

            if t > self.start_joint_set_time:
                self.hold_cog_x = self.current_cog_x
                self.hold_cog_y = self.current_cog_y
                self.hold_cog_yaw = self.current_cog_yaw

                rospy.loginfo(
                    "Start joint pose reached. "
                    "q1=%.3f, q2=%.3f, q3=%.3f. Moving to target altitude.",
                    self.q1_start,
                    self.q2_start,
                    self.q3_start
                )

                self.change_state("GO_TARGET_ALTITUDE")

        elif self.state == "GO_TARGET_ALTITUDE":
            # trajectory の開始姿勢を維持しながら目標高度へ移動
            self.publish_joint_command(
                self.q1_start,
                self.q2_start,
                self.q3_start
            )

            self.publish_z_velocity_command()

            if self.is_altitude_stable():
                if self.stable_start_time is None:
                    self.stable_start_time = now

                stable_elapsed = (now - self.stable_start_time).to_sec()

                rospy.loginfo_throttle(
                    1.0,
                    "altitude stable candidate: z=%.3f, target_z=%.3f, vz=%.3f, stable_elapsed=%.2f",
                    self.current_z,
                    self.target_z,
                    self.current_vz,
                    stable_elapsed
                )

                if stable_elapsed > self.stable_time:
                    self.motion_start_time = now

                    self.hold_cog_x = self.current_cog_x
                    self.hold_cog_y = self.current_cog_y
                    self.hold_cog_yaw = self.current_cog_yaw

                    rospy.loginfo(
                        "Altitude stabilized. Starting joint-specific q1-q2-q3 motion. "
                        "period=%.3f s, duration=%.3f s. "
                        "Holding COG at x=%.3f, y=%.3f, yaw=%.3f, target_z=%.3f",
                        self.motion_period,
                        self.motion_duration,
                        self.hold_cog_x,
                        self.hold_cog_y,
                        self.hold_cog_yaw,
                        self.target_z
                    )

                    self.change_state("SLOW_JOINT_MOTION")
            else:
                self.stable_start_time = None

        elif self.state == "SLOW_JOINT_MOTION":
            # 目標高度と COG x,y,yaw を保持しながら q1,q2,q3 を変化
            self.publish_cog_xyz_yaw_position_command()

            t = (now - self.motion_start_time).to_sec()
            q1, q2, q3 = self.slow_joint_trajectory(t)

            self.publish_joint_command(q1, q2, q3)

            if t > self.motion_duration:
                self.q1_final = q1
                self.q2_final = q2
                self.q3_final = q3

                rospy.loginfo(
                    "joint-specific q1-q2-q3 motion finished. Holding final pose: "
                    "q1=%.3f, q2=%.3f, q3=%.3f",
                    self.q1_final,
                    self.q2_final,
                    self.q3_final
                )

                self.change_state("HOLD")

        elif self.state == "HOLD":
            # 最終姿勢と目標高度を維持
            self.publish_cog_xyz_yaw_position_command()

            self.publish_joint_command(
                self.q1_final,
                self.q2_final,
                self.q3_final
            )


if __name__ == "__main__":
    node = CeilingEffectRunNode()
    rospy.spin()