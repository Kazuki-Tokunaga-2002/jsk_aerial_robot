#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>

#include <kdl/frames.hpp>

#include <Eigen/Dense>

#include <hydrus/hydrus_robot_model.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

class Leg5CircleRobotModelIK
{
public:
  Leg5CircleRobotModelIK()
    : nh_(),
      pnh_("~"),
      tf_listener_(ros::Duration(10.0)),
      initialized_(false),
      have_joint_state_(false)
  {
    loadParams();

    prepareRobotDescription();

    robot_model_ = std::make_shared<HydrusRobotModel>(true);

    ROS_INFO_STREAM("HydrusRobotModel getJointNum() = " << robot_model_->getJointNum());
    ROS_INFO_STREAM("joint_names size = " << joint_names_.size());
    for (const auto& name : joint_names_)
    {
      ROS_INFO_STREAM("command joint: " << name);
    }

    joint_state_sub_ = nh_.subscribe(
      joint_state_topic_,
      1,
      &Leg5CircleRobotModelIK::jointStateCallback,
      this
    );

    joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_cmd_topic_, 1);
    nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>(nav_topic_, 1);

    leg5_ref_root_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/hydrus/debug/leg5_ref_root", 1);
    leg5_now_root_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/hydrus/debug/leg5_now_root", 1);
    leg5_ref_world_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/hydrus/debug/leg5_ref_world", 1);
    leg5_now_world_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/hydrus/debug/leg5_now_world", 1);

    root_initial_world_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/hydrus/debug/root_initial_world", 1);
    root_now_world_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/hydrus/debug/root_now_world", 1);
    root_error_world_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "/hydrus/debug/root_error_world", 1);

    leg5_error_root_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "/hydrus/debug/leg5_error_root", 1);
    
    leg5_error_world_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "/hydrus/debug/leg5_error_world", 1);
    
    ROS_INFO("[leg5_circle_robot_model_ik] constructed");
  }

  void spin()
  {
    ros::Rate rate(rate_);

    while (ros::ok())
    {
      ros::spinOnce();

      if (!initialized_)
      {
        tryInitialize();
      }
      else
      {
        update();
      }

      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber joint_state_sub_;
  ros::Publisher joint_cmd_pub_;
  ros::Publisher nav_pub_;

  ros::Publisher leg5_ref_root_pub_;
  ros::Publisher leg5_now_root_pub_;
  ros::Publisher leg5_ref_world_pub_;
  ros::Publisher leg5_now_world_pub_;
  ros::Publisher root_initial_world_pub_;
  ros::Publisher root_now_world_pub_;
  ros::Publisher root_error_world_pub_;
  ros::Publisher leg5_error_root_pub_;
  ros::Publisher leg5_error_world_pub_;
  
  tf::TransformListener tf_listener_;
  tf::StampedTransform world_T_root_initial_;

  std::shared_ptr<HydrusRobotModel> robot_model_;

  std::mutex joint_mutex_;
  std::map<std::string, double> joint_map_;

  bool initialized_;
  bool have_joint_state_;

  ros::Time start_time_;
  ros::Time prev_update_time_;

  std::string joint_state_topic_;
  std::string joint_cmd_topic_;
  std::string nav_topic_;

  std::string world_frame_;
  std::string tf_root_frame_;
  std::string tf_yaw_frame_;

  bool use_initial_yaw_;

  std::string ee_name_;

  std::vector<std::string> joint_names_;
  std::vector<double> q_min_;
  std::vector<double> q_max_;

  double rate_;

  double circle_period_;
  double circle_center_offset_x_;
  double circle_center_offset_y_;
  double circle_center_offset_z_;

  double ellipse_radius_x_;
  double ellipse_radius_y_;

  Eigen::Vector3d w_r_leg5_center_;

  double yaw_ref_;
  double initial_root_yaw_;

  // Kinematic parameters
  double L0_, L1_, L2_, L3_;
  double q2_fixed_;
  
  // Precomputed virtual link parameters
  double Cx_, Cy_, C_norm_, phi_;

  double max_joint_vel_;
  double root_feedback_gain_;
  Eigen::Vector3d root_r_leg5_center_;
  Eigen::VectorXd q_last_;

private:
  void loadParams()
  {
    pnh_.param<std::string>("joint_state_topic", joint_state_topic_, "/hydrus/joint_states");
    pnh_.param<std::string>("joint_cmd_topic", joint_cmd_topic_, "/hydrus/joints_ctrl");
    pnh_.param<std::string>("nav_topic", nav_topic_, "/hydrus/uav/nav");

    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("tf_root_frame", tf_root_frame_, "hydrus/root");
    pnh_.param<std::string>("tf_yaw_frame", tf_yaw_frame_, "hydrus/fc");
    pnh_.param<bool>("use_initial_yaw", use_initial_yaw_, true);
    pnh_.param<std::string>("ee_name", ee_name_, "leg5");

    pnh_.param<double>("rate", rate_, 50.0);

    pnh_.param<double>("circle_period", circle_period_, 120.0);
    pnh_.param<double>("ellipse_radius_x", ellipse_radius_x_, 0.05);
    pnh_.param<double>("ellipse_radius_y", ellipse_radius_y_, 0.02);
    pnh_.param<double>("circle_center_offset_x", circle_center_offset_x_, 0.0);
    pnh_.param<double>("circle_center_offset_y", circle_center_offset_y_, 0.0);
    pnh_.param<double>("circle_center_offset_z", circle_center_offset_z_, 0.0);

    pnh_.param<double>("yaw_ref", yaw_ref_, 0.0);

    // Link lengths for Analytical IK
    pnh_.param<double>("L0", L0_, 0.58);
    pnh_.param<double>("L1", L1_, 0.575);
    pnh_.param<double>("L2", L2_, 0.625);
    pnh_.param<double>("L3", L3_, 0.55884);
    pnh_.param<double>("q2_fixed", q2_fixed_, 0.6);

    pnh_.param<double>("max_joint_vel", max_joint_vel_, 0.05);
    pnh_.param<double>("root_feedback_gain", root_feedback_gain_, 0.3);
    
    joint_names_ = {"joint1", "joint2", "joint3"};
    q_min_ = {0.0, 0.0, 0.0};
    q_max_ = {1.57, 0.60, 1.57};

    pnh_.getParam("joint_names", joint_names_);
    pnh_.getParam("q_min", q_min_);
    pnh_.getParam("q_max", q_max_);

    q_last_ = Eigen::VectorXd::Zero(joint_names_.size());
  }

  // Helper function to normalize angle to [-pi, pi]
  double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void prepareRobotDescription()
  {
    std::string robot_description;
    if (nh_.getParam("/robot_description", robot_description)) return;
    if (nh_.getParam("/hydrus/robot_description", robot_description))
    {
      nh_.setParam("/robot_description", robot_description);
      return;
    }
    if (nh_.getParam("/hydrus/robot_description_rviz", robot_description))
    {
      nh_.setParam("/robot_description", robot_description);
      return;
    }
    ROS_FATAL("No robot_description found.");
    throw std::runtime_error("robot_description not found");
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (i < msg->position.size()) joint_map_[msg->name[i]] = msg->position[i];
    }
    have_joint_state_ = true;
  }

  bool getCurrentQ(Eigen::VectorXd& q)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (!have_joint_state_) return false;

    q = Eigen::VectorXd::Zero(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      const std::string& name = joint_names_[i];
      if (joint_map_.find(name) == joint_map_.end()) return false;
      q(i) = joint_map_[name];
    }
    return true;
  }

  void updateRobotModel(const Eigen::VectorXd& q)
  {
    sensor_msgs::JointState state;
    state.header.stamp = ros::Time::now();
    state.name = joint_names_;
    state.position.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) state.position[i] = q(i);
    robot_model_->updateRobotModel(state);
  }

  Eigen::Vector3d kdlToEigen(const KDL::Vector& v) const { return Eigen::Vector3d(v.x(), v.y(), v.z()); }
  Eigen::Vector3d tfToEigen(const tf::Vector3& v) const { return Eigen::Vector3d(v.x(), v.y(), v.z()); }
  tf::Vector3 eigenToTf(const Eigen::Vector3d& v) const { return tf::Vector3(v.x(), v.y(), v.z()); }

  void printSegmentsTfOnce()
  {
    ROS_WARN_STREAM("baselink = " << robot_model_->getBaselinkName());
  }

  bool getRootToEE(Eigen::Vector3d& root_r_ee)
  {
    const auto seg_frames = robot_model_->getSegmentsTf();
    if (seg_frames.find(ee_name_) == seg_frames.end()) return false;
    root_r_ee = kdlToEigen(seg_frames.at(ee_name_).p);
    return true;
  }



  bool lookupCurrentRootTransform(tf::StampedTransform& world_T_root_now)
  {
    try
    {
      tf_listener_.lookupTransform(world_frame_, tf_root_frame_, ros::Time(0), world_T_root_now);
      return true;
    }
    catch (tf::TransformException& ex)
    {
      return false;
    }
  }

  tf::Vector3 rootToWorldInitial(const Eigen::Vector3d& root_p)
  {
    tf::Vector3 p_root(root_p.x(), root_p.y(), root_p.z());
    return world_T_root_initial_.getOrigin() + world_T_root_initial_.getBasis() * p_root;
  }

  tf::Vector3 rootToWorldWithTransform(const Eigen::Vector3d& root_p, const tf::StampedTransform& world_T_root)
  {
    tf::Vector3 p_root(root_p.x(), root_p.y(), root_p.z());
    return world_T_root.getOrigin() + world_T_root.getBasis() * p_root;
  }

  void publishPoint(const ros::Publisher& pub, const std::string& frame_id, const Eigen::Vector3d& p)
  {
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.point.x = p.x(); msg.point.y = p.y(); msg.point.z = p.z();
    pub.publish(msg);
  }

  void publishVector(const ros::Publisher& pub, const std::string& frame_id, const Eigen::Vector3d& v)
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.vector.x = v.x(); msg.vector.y = v.y(); msg.vector.z = v.z();
    pub.publish(msg);
  }

  void tryInitialize()
  {
    Eigen::VectorXd q_current;
    if (!getCurrentQ(q_current)) return;

    if (q_current.size() >= 2) q_current(1) = q2_fixed_;
    applyJointLimits(q_current);

    try
    {
      tf_listener_.waitForTransform(world_frame_, tf_root_frame_, ros::Time(0), ros::Duration(1.0));
      tf_listener_.lookupTransform(world_frame_, tf_root_frame_, ros::Time(0), world_T_root_initial_);

      if (use_initial_yaw_)
      {
        tf::StampedTransform world_T_yaw_initial;
        tf_listener_.waitForTransform(world_frame_, tf_yaw_frame_, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform(world_frame_, tf_yaw_frame_, ros::Time(0), world_T_yaw_initial);
        yaw_ref_ = tf::getYaw(world_T_yaw_initial.getRotation());
      }
    }
    catch (tf::TransformException& ex)
    {
      return;
    }

    // Precompute virtual link constant parameters for Analytical IK
    Cx_ = L1_ + L2_ * std::cos(q2_fixed_);
    Cy_ = L2_ * std::sin(q2_fixed_);
    C_norm_ = std::hypot(Cx_, Cy_);
    phi_ = std::atan2(Cy_, Cx_);

    try { updateRobotModel(q_current); } catch (...) { return; }

    printSegmentsTfOnce();

    Eigen::Vector3d root_r_leg5;
    if (!getRootToEE(root_r_leg5)) return;

    root_r_leg5_center_ = root_r_leg5;
    root_r_leg5_center_.x() += circle_center_offset_x_;
    root_r_leg5_center_.y() += circle_center_offset_y_;
    root_r_leg5_center_.z() += circle_center_offset_z_;

    tf::Vector3 w_r_leg5_initial_tf = rootToWorldInitial(root_r_leg5);
    Eigen::Vector3d w_r_leg5_initial = tfToEigen(w_r_leg5_initial_tf);

    w_r_leg5_center_ = w_r_leg5_initial;
    w_r_leg5_center_.x() += circle_center_offset_x_;
    w_r_leg5_center_.y() += circle_center_offset_y_;
    w_r_leg5_center_.z() += circle_center_offset_z_;

    q_last_ = q_current;
    start_time_ = ros::Time::now();
    prev_update_time_ = ros::Time::now();
    initialized_ = true;

    Eigen::Vector3d w_r_root_initial = tfToEigen(world_T_root_initial_.getOrigin());
    publishPoint(root_initial_world_pub_, world_frame_, w_r_root_initial);

    ROS_INFO_STREAM("Initialized leg5 circle IK node (Analytical)");
    ROS_INFO_STREAM("q2_fixed = " << q2_fixed_);
    ROS_INFO_STREAM("C_norm = " << C_norm_ << ", phi = " << phi_);
  }

  void update()
  {
    const ros::Time now = ros::Time::now();
    double dt = (now - prev_update_time_).toSec();
    prev_update_time_ = now;
    if (dt <= 0.0 || dt > 1.0) dt = 1.0 / rate_;

    const double t = (now - start_time_).toSec();
    const double omega = 2.0 * M_PI / circle_period_;

    tf::StampedTransform world_T_root_now;
    bool have_root_now = lookupCurrentRootTransform(world_T_root_now);

    Eigen::Vector3d w_r_root_initial = tfToEigen(world_T_root_initial_.getOrigin());
    Eigen::Vector3d w_r_root_cmd = w_r_root_initial;
    Eigen::Vector3d w_r_root_now = w_r_root_initial;
    Eigen::Vector3d root_error_world = Eigen::Vector3d::Zero();

    if (have_root_now)
    {
      w_r_root_now = tfToEigen(world_T_root_now.getOrigin());
      root_error_world = w_r_root_now - w_r_root_initial;
    }

    Eigen::Vector3d w_r_leg5_des = w_r_leg5_center_;
    w_r_leg5_des.x() += ellipse_radius_x_ * (std::cos(omega * t) - 1.0);
    w_r_leg5_des.y() += ellipse_radius_y_ * std::sin(omega * t);

    Eigen::Vector3d root_r_leg5_ref = root_r_leg5_center_;

    if (have_root_now)
    {
      tf::Vector3 w_r_leg5_des_tf(w_r_leg5_des.x(), w_r_leg5_des.y(), w_r_leg5_des.z());
      
      // Task Space Compensation: Use actual root position (Origin) to absorb drift
      tf::Vector3 root_r_leg5_ref_tf =
        world_T_root_now.getBasis().inverse()
        * (w_r_leg5_des_tf - world_T_root_now.getOrigin());

      root_r_leg5_ref = tfToEigen(root_r_leg5_ref_tf);
    }
    else
    {
      tf::Vector3 w_r_leg5_des_tf(w_r_leg5_des.x(), w_r_leg5_des.y(), w_r_leg5_des.z());
      tf::Vector3 root_r_leg5_ref_tf =
        world_T_root_initial_.getBasis().inverse()
        * (w_r_leg5_des_tf - world_T_root_initial_.getOrigin());
      root_r_leg5_ref = tfToEigen(root_r_leg5_ref_tf);
    }

    // Solve Analytical IK
    Eigen::VectorXd q_ik;
    bool ik_ok = solveIK(root_r_leg5_ref, q_last_, q_ik);

    Eigen::VectorXd q_cmd = limitJointVelocity(q_last_, q_ik, dt);
    applyJointLimits(q_cmd);

    q_last_ = q_cmd;

    try { updateRobotModel(q_cmd); } catch (...) { return; }

    publishJointCommand(q_cmd);

    Eigen::Vector3d root_r_leg5_now;
    if (!getRootToEE(root_r_leg5_now)) return;

    Eigen::Vector3d leg5_error_root = root_r_leg5_ref - root_r_leg5_now;

    publishPoint(leg5_ref_root_pub_, tf_root_frame_, root_r_leg5_ref);
    publishPoint(leg5_now_root_pub_, tf_root_frame_, root_r_leg5_now);
    publishVector(leg5_error_root_pub_, tf_root_frame_, leg5_error_root);

    Eigen::Vector3d w_r_leg5_ref = w_r_leg5_des;
    publishPoint(leg5_ref_world_pub_, world_frame_, w_r_leg5_ref);

    Eigen::Vector3d w_r_leg5_now;
    if (have_root_now)
    {
      tf::Vector3 w_r_leg5_now_tf = rootToWorldWithTransform(root_r_leg5_now, world_T_root_now);
      w_r_leg5_now = tfToEigen(w_r_leg5_now_tf);
      Eigen::Vector3d leg5_error_world = w_r_leg5_ref - w_r_leg5_now;

      publishPoint(leg5_now_world_pub_, world_frame_, w_r_leg5_now);
      publishVector(leg5_error_world_pub_, world_frame_, leg5_error_world);
      publishPoint(root_now_world_pub_, world_frame_, w_r_root_now);
      publishVector(root_error_world_pub_, world_frame_, root_error_world);
    }
    
    publishRootNavCommand(w_r_root_cmd);
  }

  bool solveIK(const Eigen::Vector3d& root_r_ee_ref, const Eigen::VectorXd& q_init, Eigen::VectorXd& q_out)
  {
    q_out = q_init;
    if (q_out.size() != 3) q_out = Eigen::VectorXd::Zero(3);

    // Remove L0 offset from x target
    double x = root_r_ee_ref.x() - L0_;
    double y = root_r_ee_ref.y();

    double r_sq = x * x + y * y;
    double r = std::sqrt(r_sq);

    // Cosine rule for virtual elbow angle
    double D = (r_sq - C_norm_ * C_norm_ - L3_ * L3_) / (2.0 * C_norm_ * L3_);
    
    bool reachable = true;
    if (D > 1.0) { D = 1.0; reachable = false; }
    else if (D < -1.0) { D = -1.0; reachable = false; }

    double acos_D = std::acos(D);
    
    // Calculate two possible q3 solutions
    double q3_plus  = phi_ - q2_fixed_ + acos_D;
    double q3_minus = phi_ - q2_fixed_ - acos_D;

    q3_plus = normalizeAngle(q3_plus);
    q3_minus = normalizeAngle(q3_minus);

    // Pick the solution closest to the previous q3
    double diff_plus = std::abs(normalizeAngle(q3_plus - q_init(2)));
    double diff_minus = std::abs(normalizeAngle(q3_minus - q_init(2)));
    double q3 = (diff_plus < diff_minus) ? q3_plus : q3_minus;

    // Forward kinematics of virtual link to find V
    double vx = Cx_ + L3_ * std::cos(q2_fixed_ + q3);
    double vy = Cy_ + L3_ * std::sin(q2_fixed_ + q3);

    // Calculate q1
    double q1 = std::atan2(y, x) - std::atan2(vy, vx);
    q1 = normalizeAngle(q1);

    q_out(0) = q1;
    q_out(1) = q2_fixed_;
    q_out(2) = q3;

    if (!reachable)
    {
      ROS_WARN_THROTTLE(1.0, "IK target out of workspace. Clamping to boundary.");
    }

    return reachable;
  }

  void applyJointLimits(Eigen::VectorXd& q)
  {
    for (int i = 0; i < q.size(); ++i)
    {
      if (i < static_cast<int>(q_min_.size())) q(i) = std::max(q(i), q_min_[i]);
      if (i < static_cast<int>(q_max_.size())) q(i) = std::min(q(i), q_max_[i]);
    }
    if (q.size() >= 2) q(1) = q2_fixed_;
  }

  Eigen::VectorXd limitJointVelocity(const Eigen::VectorXd& q_prev, const Eigen::VectorXd& q_target, double dt)
  {
    if (q_prev.size() != q_target.size()) return q_target;
    Eigen::VectorXd q_limited = q_target;
    const double max_delta = max_joint_vel_ * dt;

    for (int i = 0; i < q_target.size(); ++i)
    {
      if (i == 1)
      {
        q_limited(i) = q2_fixed_;
        continue;
      }

      // Handle angle wrapping correctly for velocity limits
      double delta = normalizeAngle(q_target(i) - q_prev(i));

      if (delta > max_delta) delta = max_delta;
      else if (delta < -max_delta) delta = -max_delta;

      q_limited(i) = normalizeAngle(q_prev(i) + delta);
    }
    if (q_limited.size() >= 2) q_limited(1) = q2_fixed_;
    
    return q_limited;
  }

  void publishJointCommand(const Eigen::VectorXd& q)
  {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = joint_names_;
    msg.position.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) msg.position[i] = q(i);
    joint_cmd_pub_.publish(msg);
  }

  void publishRootNavCommand(const Eigen::Vector3d& w_r_root_ref)
  {
    aerial_robot_msgs::FlightNav msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = world_frame_;

    msg.control_frame = aerial_robot_msgs::FlightNav::WORLD_FRAME;
    msg.target = aerial_robot_msgs::FlightNav::ROOT;

    msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
    msg.target_pos_x = w_r_root_ref.x();
    msg.target_pos_y = w_r_root_ref.y();

    msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
    msg.target_pos_z = w_r_root_ref.z();

    msg.roll_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
    msg.pitch_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
    msg.yaw_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
    msg.target_yaw = q_last_(0) + q_last_(1);

    msg.target_vel_x = 0.0; msg.target_vel_y = 0.0; msg.target_vel_z = 0.0;
    msg.target_acc_x = 0.0; msg.target_acc_y = 0.0;
    msg.target_omega_x = 0.0; msg.target_omega_y = 0.0; msg.target_omega_z = 0.0;

    nav_pub_.publish(msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg5_circle_robot_model_ik_node");

  try
  {
    Leg5CircleRobotModelIK node;
    node.spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Exception in leg5_circle_robot_model_ik_node: " << e.what());
    return 1;
  }
  return 0;
}
