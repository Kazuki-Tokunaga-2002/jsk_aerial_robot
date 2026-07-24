#!/usr/bin/env python3
import skrobot
from skrobot.model import RobotModel
from skrobot.coordinates import make_coords
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

# Load from file path
robot = RobotModel()
robot.load_urdf_file("/home/tokunaga/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/hydrus/robots/quad/tilt_0deg_ce_15inch_202604/robot.urdf")

robot.joint1.joint_angle(1.2)
robot.joint2.joint_angle(0.6)
robot.joint3.joint_angle(1.2)

print(robot.leg5)

ee_end_coords = skrobot.coordinates.CascadedCoords(parent=robot.leg5, name='leg5_coords')
move_target = ee_end_coords
print(move_target)

link_list = [
    robot.link1,
    robot.link2,
    robot.link3,
    robot.link4]
for i in range(len(link_list)):
    print(link_list[i])
print("links:")
for link in robot.link_list:
    print(link.name)
# joint_list = [rotor1 joint1 rotor2 joint2 rotor3 joint3 rotor4]
for joint in robot.joint_list:
    print(joint.name)

print("leg5 world pos:", robot.leg5.worldpos())
print("root world pos:", robot.root.worldpos())
print("root world rot:\n", robot.root.worldrot())

# rootの位置をworld座標系で変更.rotは指示を与えないと単位行列．
new_pos = np.array([1.5, 2.0, 1.0])
new_c = make_coords(pos=new_pos)
robot.root.newcoords(new_c)

for i in range(len(link_list)):
    print(link_list[i])
print("leg5 world pos:", robot.leg5.worldpos())
print("root world pos:", robot.root.worldpos())
print("root world rot:\n", robot.root.worldrot())

# 流れ
#1. root_r_ee(t+Δt)=(w_R_root(t))^T(w_r_ee(t+Δt)-w_r_root(const))
#2. q(t+Δt)=IK(root_r_ee(t+Δt)
#3. q(t+Δt)を仮に反映したときの，centroidを取得．w_r_cog(t+Δt)
#4. q(t+Δt),w_r_cog(t+Δt)を送る

#1. root_r_ee_in_rootの取得方法(API利用)
root_r_ee_in_root = robot.root.inverse_transform_vector(robot.leg5.worldpos())
print("root_r_ee_in_root(API):", root_r_ee_in_root)

#1. root_r_ee_in_rootの取得方法(座標変換を自分で計算)
world_r_root_in_world = robot.root.worldpos()
world_R_root_in_world = robot.root.worldrot()
world_r_ee_in_world = robot.leg5.worldpos()
root_r_ee_in_root = world_R_root_in_world.T.dot(world_r_ee_in_world - world_r_root_in_world)
print("root_r_ee_in_root:", root_r_ee_in_root)

#2. IKを解く(0.3<=joint1, joint3<=1.47, joint2=0.6)
#2.1 制限
print(robot.joint1.min_angle, robot.joint1.max_angle)
robot.joint1.min_angle=0.3
robot.joint1.max_angle=1.47
print("joint1 limits:", robot.joint1.min_angle, robot.joint1.max_angle)
robot.joint2.min_angle=0.6
robot.joint2.max_angle=0.6
print("joint2 limits:", robot.joint2.min_angle, robot.joint2.max_angle)
robot.joint3.min_angle=0.3
robot.joint3.max_angle=1.47
print("joint3 limits:", robot.joint3.min_angle, robot.joint3.max_angle)

#2.2 IKを解く
target_coords = skrobot.coordinates.CascadedCoords(pos=[1.0, 1.4, 0])
q = robot.inverse_kinematics(target_coords,
                             link_list=link_list,
                             move_target=move_target,
                             position_mask=[1, 1, 0],
                             rotation_mask=False)
#3. IKで得たqでjointを更新
robot.joint1.joint_angle(q[1])
robot.joint1.joint_angle(q[3])
robot.joint1.joint_angle(q[5])

#4. world_r_cogの取得方法(API利用)
cog_in_world = robot.centroid()
print("world_r_cog_in_world(API):", cog_in_world)
#4. world_r_cogの取得方法(座標変換を自分で計算)->cogはcentroid()でしか取れないので計算不要
robot.joint1.joint_angle(0.6)
robot.joint2.joint_angle(0.6)
robot.joint3.joint_angle(0.6)
cog_in_world = robot.centroid()
print("world_r_cog_in_world(API):", cog_in_world)

#5. q(t+dt),world_r_cog(t+dt)をpubする

# jointの指令
def make_joint_state_msg(q_cmd):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ["joint1", "joint2", "joint3"]
    msg.position = [float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2])]
    msg.velocity = []
    msg.effort = []
    return msg

# FlightNavの指令
def make_flight_nav_msg(world_r_cog_cmd):
    msg = FlightNav()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    # world座標系でCOG位置を指令する
    msg.control_frame = FlightNav.WORLD_FRAME
    msg.target = FlightNav.COG
    # x, y は位置制御
    msg.pos_xy_nav_mode = FlightNav.POS_MODE
    msg.target_pos_x = float(world_r_cog_cmd[0])
    msg.target_pos_y = float(world_r_cog_cmd[1])
    msg.target_vel_x = 0.0
    msg.target_vel_y = 0.0
    msg.target_acc_x = 0.0
    msg.target_acc_y = 0.0
    # z も位置制御
    msg.pos_z_nav_mode = FlightNav.POS_MODE
    msg.target_pos_z = float(world_r_cog_cmd[2])
    msg.target_vel_z = 0.0
    msg.target_pos_diff_z = 0.0
    # roll, pitch はここでは直接指令しない
    msg.roll_nav_mode = FlightNav.NO_NAVIGATION
    msg.pitch_nav_mode = FlightNav.NO_NAVIGATION
    msg.target_roll = 0.0
    msg.target_pitch = 0.0
    msg.target_omega_x = 0.0
    msg.target_omega_y = 0.0
    # yaw は??
    msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
    msg.target_yaw = 0.0
    msg.target_omega_z = 0.0
    return msg


# world_r_ee_ref_in_world(t)の例
def world_r_ee_ref_in_world(t):
    # 例: 円運動
    radius = 0.02
    center = np.array([1.0, 1.4, 0])
    return center + radius * np.array([np.cos(t), np.sin(t), 0])

# IKを解く関数
def solve_ik(t):
    target_coords = skrobot.coordinates.CascadedCoords(pos=world_r_ee_ref_in_world(t))
    q = robot.inverse_kinematics(target_coords,
                                 link_list=link_list,
                                 move_target=move_target,
                                 position_mask=[1, 1, 0],
                                 rotation_mask=False)
    return q

# IKで得たqをrobotに反映する関数
def apply_q(q):
    robot.joint1.joint_angle(q[1])
    robot.joint2.joint_angle(q[3])
    robot.joint3.joint_angle(q[5])
        
    q_cmd = np.array([q[1], q[3], q[5]])
    return q_cmd

def main():
    global joint_pub, nav_pub

    rospy.init_node("skr_ik_test")

    joint_pub = rospy.Publisher("/hydrus/joints_ctrl", JointState, queue_size=1)
    nav_pub = rospy.Publisher("/hydrus/uav/nav", FlightNav, queue_size=1)

    rate = rospy.Rate(100)  # 100Hz
    start_time = rospy.Time.now()

    loop_count = 0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        t = (current_time - start_time).to_sec()

        #1. root_r_ee_in_rootの取得方法(API利用)
        root_r_ee_in_root = robot.root.inverse_transform_vector(world_r_ee_ref_in_world(t))
        print("root_r_ee_in_root(API):", root_r_ee_in_root)
        
        #2. IKを解く(0.3<=joint1, joint3<=1.47, joint2=0.6)
        q = solve_ik(t)
        
        #3. IKで得たqでjointを更新
        q_cmd = apply_q(q)

        #4. world_r_cogの取得方法(API利用)
        cog_in_world = robot.centroid()
        print("world_r_cog_in_world(API):", cog_in_world)
        
        #5. q(t+dt),world_r_cog(t+dt)をpub
        nav_msg = make_flight_nav_msg(cog_in_world)
        nav_msg.header.stamp = rospy.Time.now()
        nav_pub.publish(nav_msg)

        if loop_count % 2 == 0:  
            joint_msg = make_joint_state_msg(q_cmd)
            joint_msg.header.stamp = rospy.Time.now()
            joint_pub.publish(joint_msg)

        loop_count += 1    
        rate.sleep()

if __name__ == "__main__":
    main()

# target_coords = skrobot.coordinates.CascadedCoords(pos=[1.0, 1.4, 0])
# q = robot.inverse_kinematics(target_coords,
#                              link_list=link_list,
#                              move_target=move_target,
#                              position_mask=[1, 1, 0],
#                              rotation_mask=False)
# print(q)
# print(len(q))
# print(f"q full: {q}")
# print(f"q size: {len(q)}")
# # IK後に現在の関節角度を取得
# print(f"joint1 angle: {robot.joint1.joint_angle()}")
# print(f"joint2 angle: {robot.joint2.joint_angle()}")
# print(f"joint3 angle: {robot.joint3.joint_angle()}")

# IK analytical solution: q(t)=IK(root_r_ee_in_root(t))
# def solve_q1_q3(x, y, L1, L2, L3, L4, q2=q2_fixed):
#     xp = x - L1
#     yp = y  
#     M = np.sqrt((L2 + L3 * np.cos(q2))**2 + (L3 * np.sin(q2))**2)
#     beta = np.arctan2(L3 * np.sin(q2), L2 + L3 * np.cos(q2))
#     r2 = xp**2 + yp**2
#     D = (r2 - M**2 - L4**2) / (2 * M * L4)
#     if D < -1.0 or D > 1.0:
#         return []  # 到達不能
#     D = np.clip(D, -1.0, 1.0)
#     solutions = []
#     for gamma in [np.arccos(D), -np.arccos(D)]:
#         theta1 = np.arctan2(yp, xp) - np.arctan2(
#             L4 * np.sin(gamma),
#             M + L4 * np.cos(gamma)
#         )
#         q1 = theta1 - beta
#         q3 = gamma - q2 + beta
#         solutions.append((q1, q2, q3))
# return solutions

# def solve_q1_q3_with_limits(x, y, L1, L2, L3, L4, q2=q2_fixed):
#     sols = solve_q1_q3(x, y, L1, L2, L3, L4, q2)
#     valid = []
#     for q1, q2, q3 in sols:
#         if 0.0 <= q1 <= 1.57 and 0.0 <= q3 <= 1.57:
#             valid.append((q1, q2, q3))
# return valid


# get world_r_cog
# world_r_cog_in_world = world_r_root_in_world + root.worldrot().dot(root_r_ee_in_root)

# root_pos_fixed = robot.root.worldpos() #world_r_root_in_world 
# ee_initial_pos = robot.leg5.worldpos() # world_r_ee_in_world 
# q2_fixed = 0.6

# time_steps = np.linspace(0, 2*np.pi, 100)
# rx = 0.02
# ry = 0.02

# trajectory = np.array([
#     [ee_initial_pos[0] + rx * np.cos(t), 
#      ee_initial_pos[1] + ry * np.sin(t),
#      ee_initial_pos[2]]
#     for t in time_steps
# ])

# results = []
# for world_r_ee_in_world in trajectory:
    
#     robot.joint2.joint_angle(q2_fixed)  

#     #1. get root_r_ee
#     root_r_ee_in_world = world_r_ee_in_world - root_pos_fixed
#     root_r_ee_in_root = robot.root.worldrot().T.dot(root_r_ee_in_world)

#     #2. solve IK q(t)=IK(root_r_ee_in_root(t))
#     target_coords = skrobot.coordinates.Coordinates(root_r_ee_in_root, [0, 0, 0])
#     q = robot.inverse_kinematics(
#         target_coords,
#         link_list=link_list,
#         move_target=move_target,
#         position_mask=[1, 1, 0],
#         rotation_mask=False)

# if q is not False: 
#     #3. change q1, q3 of robot
#     robot.joint1.joint_angle(q[1])
#     robot.joint2.joint_angle(q[3])
#     robot.joint3.joint_angle(q[5])

#     #4. get world_r_cog
#     robot.forward_kinematics(q)
#     world_r_cog_in_world = robot.centroid()

#     results.append({'q': q, 'cog': world_r_cog_in_world})  

# def main():

# """
# - URDFをみてもらって，floating baseな関節(7自由度)の与え方をきく（むりかも）
# - 多分move_targetの設定がミスっているので先生に聞く
# - target coordsも同様
# - root固定でIKを解く
# - IKで得たqでjoint*を更新
# - root_r_com_in_rootを取る方法を聞く
# """
