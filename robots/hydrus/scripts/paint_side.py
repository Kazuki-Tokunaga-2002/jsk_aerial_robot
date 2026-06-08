#!/usr/bin/env python
from __future__ import print_function

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

        # # 高度安定判定
        # self.z_threshold = rospy.get_param("~z_threshold", 0.10)
        # self.vz_threshold = rospy.get_param("~vz_threshold", 0.08)
        # self.stable_time = rospy.get_param("~stable_time", 1.5)

        # # 目標高度へ移動するときの速度制御
        # self.kp_z = rospy.get_param("~kp_z", 0.10)
        # self.max_vz = rospy.get_param("~max_vz", 0.05)

        # ===== 関節角度パラメータ =====
        self.q1_const = rospy.get_param("~q1_const", 1.40)
        self.q2_const = rospy.get_param("~q2_const", 0.60)

        self.q3_start = rospy.get_param("~q3_start", 1.20)
        self.q3_goal = rospy.get_param("~q3_goal", 0.80)

        self.motion_duration = rospy.get_param("~motion_duration", 40.0)

        # 初期姿勢へ移動するときの関節速度
        self.joint_set_speed = rospy.get_param("~joint_set_speed", 0.03)
        self.min_joint_set_time = rospy.get_param("~min_joint_set_time", 1.0)

        # ===== 現在値 =====
        self.current_cog_x = None
        self.current_cog_y = None
        self.current_cog_z = None
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
        self.hold_cog_z = None
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
        rospy.loginfo("q1 const = %.3f", self.q1_const)
        rospy.loginfo("q2 const = %.3f", self.q2_const)
        rospy.loginfo("q3 start = %.3f, q3 goal = %.3f", self.q3_start, self.q3_goal)
        rospy.loginfo("joint_set_speed = %.3f [rad/s]", self.joint_set_speed)
        rospy.loginfo("motion_duration = %.3f [s]", self.motion_duration)

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
        self.current_cog_z = msg.pose.pose.position.z

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

    def setup_start_joint_motion(self):
        self.q1_init = self.current_q1
        self.q2_init = self.current_q2
        self.q3_init = self.current_q3

        if self.joint_set_speed <= 0.0:
            rospy.logwarn("joint_set_speed <= 0. Use default 0.03 rad/s")
            self.joint_set_speed = 0.03

        max_diff = max(
            abs(self.q1_const - self.q1_init),
            abs(self.q2_const - self.q2_init),
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
            self.q1_const,
            self.q2_init,
            self.q2_const,
            self.q3_init,
            self.q3_start,
            self.start_joint_set_time
        )

    def start_joint_trajectory(self, t):
        """
        最初の姿勢合わせ:
        現在の q1, q2, q3 から
        q1_const, q2_const, q3_start へ 0.03 rad/s 相当で移動。
        """
        s = self.smooth_step(t, self.start_joint_set_time)

        q1 = self.q1_init + s * (self.q1_const - self.q1_init)
        q2 = self.q2_init + s * (self.q2_const - self.q2_init)
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
        nav_msg.target_pos_z = self.hold_cog_z

        nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        nav_msg.target_yaw = self.hold_cog_yaw

        self.nav_pub.publish(nav_msg)

    def publish_joint_command(self, q1, q2, q3):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = [q1, q2, q3]

        # velocity は「この速度で動け」という意味ではなく、
        # JointState の velocity 欄として 0.03 を入れておく。
        msg.velocity = [self.joint_set_speed, self.joint_set_speed, self.joint_set_speed]

        # effort は空
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

    def slow_joint_trajectory(self, t):
        """
        メイン実験フェーズ:
        q1, q2 は固定。
        q3 のみ q3_start -> q3_goal へ smooth step で変化。
        """
        s = self.smooth_step(t, self.motion_duration)

        q1 = self.q1_const
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_goal - self.q3_start)

        return q1, q2, q3

    def update(self, event):
        now = rospy.Time.now()

        if self.state == "WAIT_JOINT_STATE_AND_ODOM":
            if (
                self.current_q1 is not None and
                self.current_q2 is not None and
                self.current_q3 is not None and
                self.current_cog_x is not None and
                self.current_cog_y is not None and
                self.current_cog_z is not None and
                self.current_cog_yaw is not None
            ):
                rospy.loginfo(
                    "current state received: q1=%.3f, q2=%.3f, q3=%.3f, "
                    "cog_x=%.3f, cog_y=%.3f, cog_z=%.3f, cog_yaw=%.3f",
                    self.current_q1,
                    self.current_q2,
                    self.current_q3,
                    self.current_cog_x,
                    self.current_cog_y,
                    self.current_cog_z,
                    self.current_cog_yaw
                )

                # rosrun後，最初に取得できたCOGを保持目標として保存
                self.hold_cog_x = self.current_cog_x
                self.hold_cog_y = self.current_cog_y
                self.hold_cog_z = self.current_cog_z
                self.hold_cog_yaw = self.current_cog_yaw

                self.setup_start_joint_motion()
                self.change_state("SET_START_JOINTS")

            else:
                rospy.loginfo_throttle(
                    2.0,
                    "waiting for joint_states and cog odom: "
                    "q1=%s, q2=%s, q3=%s, cog_x=%s, cog_y=%s, cog_z=%s, cog_yaw=%s",
                    str(self.current_q1),
                    str(self.current_q2),
                    str(self.current_q3),
                    str(self.current_cog_x),
                    str(self.current_cog_y),
                    str(self.current_cog_z),
                    str(self.current_cog_yaw)
                )

        elif self.state == "SET_START_JOINTS":
            # 起動時のCOG x,y,z,yawを保持しながら，開始関節姿勢へ移動
            self.publish_cog_xyz_yaw_position_command()

            t = (now - self.start_joint_motion_start_time).to_sec()
            q1, q2, q3 = self.start_joint_trajectory(t)

            self.publish_joint_command(q1, q2, q3)

            if t > self.start_joint_set_time:
                rospy.loginfo(
                    "Start joint pose reached. "
                    "Start q3-only motion while holding COG: "
                    "x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                    self.hold_cog_x,
                    self.hold_cog_y,
                    self.hold_cog_z,
                    self.hold_cog_yaw
                )

                self.motion_start_time = now
                self.change_state("SLOW_JOINT_MOTION")

        elif self.state == "SLOW_JOINT_MOTION":
            # 起動時のCOG x,y,z,yawを保持しながら q3 のみ変化
            self.publish_cog_xyz_yaw_position_command()

            t = (now - self.motion_start_time).to_sec()
            q1, q2, q3 = self.slow_joint_trajectory(t)

            self.publish_joint_command(q1, q2, q3)

            if t > self.motion_duration:
                rospy.loginfo(
                    "q3-only motion finished. Holding final pose: "
                    "q1=%.3f, q2=%.3f, q3=%.3f",
                    self.q1_const,
                    self.q2_const,
                    self.q3_goal
                )

                self.change_state("HOLD")

        elif self.state == "HOLD":
            self.publish_cog_xyz_yaw_position_command()

            self.publish_joint_command(
                self.q1_const,
                self.q2_const,
                self.q3_goal
            )


if __name__ == "__main__":
    node = CeilingEffectRunNode()
    rospy.spin()
