#include <delta/control/delta_controller.h>

#include <algorithm>
#include <cmath>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

namespace
{
double clampDouble(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}
}  // namespace

DeltaController::DeltaController() : PoseLinearController(), torque_allocation_matrix_inv_pub_stamp_(0)
{
}

void DeltaController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                 double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  delta_robot_model_ = boost::dynamic_pointer_cast<DeltaRobotModel>(robot_model_);
  robot_model_for_control_ = boost::make_shared<DeltaRobotModel>();

  first_run_ = true;

  rotor_tilt_.resize(motor_num_);
  lambda_all_.resize(motor_num_, 0.0);
  ceiling_thrust_scale_.resize(motor_num_, 1.0);
  target_gimbal_angles_.resize(motor_num_, 0.0);
  nlopt_phi_nominal_.resize(motor_num_, 0.0);

  rosParamInit();

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  servo_torque_command_pub_ = nh_.advertise<spinal::ServoTorqueCmd>("servo/torque_enable", 1);
  torque_allocation_matrix_inv_pub_ =
      nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);

  target_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_cog", 1);
  nlopt_log_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/nlopt_log", 1);
  nlopt_iterations_pub_ = nh.advertise<std_msgs::Int16>("debug/nlopt_iterations", 1);
  nlopt_result_pub_ = nh.advertise<std_msgs::Int16>("debug/nlopt_result", 1);
  rotor_origin_pub_ = nh.advertise<geometry_msgs::PoseArray>("debug/rotor_origin", 1);
  rotor_normal_pub_ = nh.advertise<geometry_msgs::PoseArray>("debug/rotor_normal", 1);
  ceiling_effect_distance_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/ceiling_effect/d_i", 1);
  ceiling_effect_dbar_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/ceiling_effect/dbar_i", 1);
  ceiling_effect_theta_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/ceiling_effect/theta_i", 1);
  ceiling_effect_ratio_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/ceiling_effect/k_i", 1);
  ceiling_effect_scale_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/ceiling_effect/1_k_i", 1);

  q_mat_.resize(6, motor_num_);
  q_mat_inv_.resize(motor_num_, 6);
}

void DeltaController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();

  torque_allocation_matrix_inv_pub_stamp_ = -1;

  first_run_ = true;
}

void DeltaController::activate()
{
  ControlBase::activate();

  // enable all servos when activated
  spinal::ServoTorqueCmd servo_torque_command;
  for (int i = 0; i < robot_model_->getJointNum(); i++)
  {
    servo_torque_command.index.push_back(i);
    servo_torque_command.torque_enable.push_back(1);
  }
  servo_torque_command_pub_.publish(servo_torque_command);

  // set gimbal angles to zero
  for (int i = 0; i < motor_num_; i++)
  {
    target_gimbal_angles_.at(i) = 0.0;
  }
  sendGimbalAngles();
}

void DeltaController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");

  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_,
                   0.05);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "use_fc_for_att_control", use_fc_for_att_control_, true);
  getParam<bool>(control_nh, "linear_mode", linear_mode_, false);
  getParam<double>(control_nh, "nlopt_phi_limit", nlopt_phi_limit_, M_PI);
  getParam<double>(control_nh, "nlopt_lambda_weight", nlopt_lambda_weight_, 1.0);
  getParam<double>(control_nh, "nlopt_delta_lambda_weight", nlopt_delta_lambda_weight_, 0.0);
  getParam<double>(control_nh, "nlopt_delta_phi_weight", nlopt_delta_phi_weight_, 0.0);
  getParam<double>(control_nh, "nlopt_phi_nominal_weight", nlopt_phi_nominal_weight_, 0.0);
  getParam<double>(control_nh, "nlopt_lambda_balance_weight", nlopt_lambda_balance_weight_, 0.0);
  getParam<int>(control_nh, "ceiling_tilt", ceiling_tilt_mode_, 0);
  getParam<double>(control_nh, "ceiling_distance", ceiling_distance_, 2.0);
  getParam<double>(control_nh, "ceiling_rotor_radius", ceiling_rotor_radius_, 0.1143);
  getParam<double>(control_nh, "ceiling_effect_min_dbar", ceiling_effect_min_dbar_, 0.2);
  getParam<double>(control_nh, "ceiling_effect_max_theta", ceiling_effect_max_theta_, 0.6981317008);

  double nlopt_phi_nominal = 0.0;
  getParam<double>(control_nh, "nlopt_phi_nominal", nlopt_phi_nominal, 0.0);
  nlopt_phi_limit_ = std::max(1.0e-3, std::min(M_PI, std::abs(nlopt_phi_limit_)));
  std::fill(nlopt_phi_nominal_.begin(), nlopt_phi_nominal_.end(), nlopt_phi_nominal);
  ceiling_tilt_mode_ = std::max(0, std::min(2, ceiling_tilt_mode_));
  ceiling_rotor_radius_ = std::max(1.0e-6, ceiling_rotor_radius_);
  ceiling_effect_min_dbar_ = std::max(1.0e-3, ceiling_effect_min_dbar_);
  ceiling_effect_max_theta_ = std::max(0.0, std::min(0.5 * M_PI, ceiling_effect_max_theta_));

  /* get tilt angle of each thruster */
  auto urdf_model = robot_model_->getUrdfModel();
  auto robot_model_xml = robot_model_->getRobotModelXml("robot_description");
  for (int i = 0; i < motor_num_; i++)
  {
    std::string rotor_parent_joint_name = std::string("gimbal_link") + std::to_string(i + 1) + std::string("2") +
                                          std::string("rotor_parent") +
                                          std::to_string(i + 1);  // we assume gimbal_link*2_rotor_parent* exists
    urdf::JointConstSharedPtr joint = urdf_model.getJoint(rotor_parent_joint_name);
    urdf::Pose origin = joint->parent_to_joint_origin_transform;

    double r, p, y;
    origin.rotation.getRPY(r, p, y);

    rotor_tilt_.at(i) = p;
  }
}

bool DeltaController::update()
{
  if (!PoseLinearController::update())
    return false;

  return true;
}

void DeltaController::controlCore()
{
  PoseLinearController::controlCore();

  /* update robot model for control */
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  robot_model_for_control_->setExtraModuleMap(robot_model_->getExtraModuleMap());
  KDL::JntArray joint_positions = robot_model_->getJointPositions();
  robot_model_for_control_->updateRobotModel(joint_positions);

  /* calculate feedback term */
  calcAccFromCog();
  forceLandingProcess();

  /* calculate control input */
  wrenchAllocation();

  /* common part */
  processGimbalAngles();
  calcYawTerm();
  /* common part */

  first_run_ = false;
}

void DeltaController::processGimbalAngles()
{
  /* update robot model by calculated gimbal angle */
  auto current_gimbal_angles = delta_robot_model_->getCurrentGimbalAngles();
  const auto& joint_index_map = robot_model_->getJointIndexMap();
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray gimbal_processed_joint = robot_model_->getJointPositions();
  for (int i = 0; i < motor_num_; i++)
  {
    std::string s = std::to_string(i + 1);
    gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = current_gimbal_angles.at(i);
  }
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  /* calculate allocation matrix for realtime control */
  q_mat_ = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);
}

void DeltaController::calcYawTerm()
{
  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0;  // for reconstruct yaw control term in spinal

  int torque_allocation_matrix_inv_rows = motor_num_;

  for (unsigned int i = 0; i < torque_allocation_matrix_inv_rows; i++)
  {
    if (q_mat_inv_(i, YAW) > max_yaw_scale)
      max_yaw_scale = q_mat_inv_(i, YAW);
  }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void DeltaController::sendCmd()
{
  PoseLinearController::sendCmd();

  /* nlopt log */
  std_msgs::Float32MultiArray nlopt_log_msg;
  nlopt_log_msg.data = nlopt_log_;
  nlopt_log_pub_.publish(nlopt_log_msg);

  std_msgs::Int16 nlopt_iterations_msg;
  nlopt_iterations_msg.data = nlopt_iterations_;
  nlopt_iterations_pub_.publish(nlopt_iterations_msg);

  std_msgs::Int16 nlopt_result_msg;
  nlopt_result_msg.data = nlopt_result_;
  nlopt_result_pub_.publish(nlopt_result_msg);

  /* publish origin and normal for debug */
  geometry_msgs::PoseArray rotor_origin_msg;
  geometry_msgs::PoseArray rotor_normal_msg;
  std::vector<Eigen::Vector3d> rotor_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotor_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  for (int i = 0; i < motor_num_; i++)
  {
    geometry_msgs::Pose origin;
    origin.position.x = rotor_origin.at(i)(0);
    origin.position.y = rotor_origin.at(i)(1);
    origin.position.z = rotor_origin.at(i)(2);
    rotor_origin_msg.poses.push_back(origin);
    geometry_msgs::Pose normal;
    normal.position.x = rotor_normal.at(i)(0);
    normal.position.y = rotor_normal.at(i)(1);
    normal.position.z = rotor_normal.at(i)(2);
    rotor_normal_msg.poses.push_back(normal);
  }
  rotor_origin_pub_.publish(rotor_origin_msg);
  rotor_normal_pub_.publish(rotor_normal_msg);

  sendGimbalAngles();

  sendFourAxisCommand();

  setAttitudeGains();

  sendTorqueAllocationMatrixInv();
}

void DeltaController::sendGimbalAngles()
{
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < motor_num_; i++)
  {
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    gimbal_control_msg.name.push_back(std::string("gimbal") + std::to_string(i + 1));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

void DeltaController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  if (use_fc_for_att_control_)
    flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = lambda_all_;
  flight_command_data.ceiling_thrust_scale = calcCeilingThrustScale();
  flight_cmd_pub_.publish(flight_command_data);
}

std::vector<float> DeltaController::calcCeilingThrustScale()
{
  std_msgs::Float32MultiArray ceiling_effect_distance_msg;
  std_msgs::Float32MultiArray ceiling_effect_dbar_msg;
  std_msgs::Float32MultiArray ceiling_effect_theta_msg;
  std_msgs::Float32MultiArray ceiling_effect_ratio_msg;
  std_msgs::Float32MultiArray ceiling_effect_scale_msg;
  ceiling_effect_distance_msg.data.resize(motor_num_, 0.0);
  ceiling_effect_dbar_msg.data.resize(motor_num_, 0.0);
  ceiling_effect_theta_msg.data.resize(motor_num_, 0.0);
  ceiling_effect_ratio_msg.data.resize(motor_num_, 1.0);
  ceiling_effect_scale_msg.data.resize(motor_num_, 1.0);

  std::fill(ceiling_thrust_scale_.begin(), ceiling_thrust_scale_.end(), 1.0f);
  if (ceiling_tilt_mode_ == 0)
  {
    ceiling_effect_distance_pub_.publish(ceiling_effect_distance_msg);
    ceiling_effect_dbar_pub_.publish(ceiling_effect_dbar_msg);
    ceiling_effect_theta_pub_.publish(ceiling_effect_theta_msg);
    ceiling_effect_ratio_pub_.publish(ceiling_effect_ratio_msg);
    ceiling_effect_scale_pub_.publish(ceiling_effect_scale_msg);
    return ceiling_thrust_scale_;
  }

  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 world_rot_from_cog =
      estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
  tf::Vector3 cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);

  std::vector<Eigen::Vector3d> rotor_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotor_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  const tf::Vector3 ceiling_normal(0.0, 0.0, 1.0);

  for (int i = 0; i < motor_num_; i++)
  {
    const Eigen::Vector3d& origin_cog_eigen = rotor_origin.at(i);
    tf::Vector3 origin_cog(origin_cog_eigen.x(), origin_cog_eigen.y(), origin_cog_eigen.z());
    tf::Vector3 origin_world = cog_pos + world_rot_from_cog * origin_cog;

    double distance = ceiling_distance_ - origin_world.z();
    double dbar = distance / ceiling_rotor_radius_;
    double theta = 0.0;

    if (ceiling_tilt_mode_ == 2)
    {
      const Eigen::Vector3d& normal_cog_eigen = rotor_normal.at(i);
      tf::Vector3 normal_cog(normal_cog_eigen.x(), normal_cog_eigen.y(), normal_cog_eigen.z());
      tf::Vector3 normal_world = world_rot_from_cog * normal_cog;
      if (normal_world.length2() > 1.0e-12)
      {
        normal_world.normalize();
        double dot = std::abs(normal_world.dot(ceiling_normal));
        theta = std::acos(clampDouble(dot, 0.0, 1.0));
      }
    }

    double k = calcCeilingThrustRatio(dbar, theta);
    if (!std::isfinite(k) || k <= 1.0e-6)
    {
      ROS_WARN_THROTTLE(1.0, "[DeltaController] invalid ceiling effect ratio, use scale 1.0");
      k = 1.0;
    }

    ceiling_thrust_scale_.at(i) = 1.0f / static_cast<float>(k);
    ceiling_effect_distance_msg.data.at(i) = static_cast<float>(distance);
    ceiling_effect_dbar_msg.data.at(i) = static_cast<float>(dbar);
    ceiling_effect_theta_msg.data.at(i) = static_cast<float>(theta);
    ceiling_effect_ratio_msg.data.at(i) = static_cast<float>(k);
    ceiling_effect_scale_msg.data.at(i) = ceiling_thrust_scale_.at(i);
  }

  ceiling_effect_distance_pub_.publish(ceiling_effect_distance_msg);
  ceiling_effect_dbar_pub_.publish(ceiling_effect_dbar_msg);
  ceiling_effect_theta_pub_.publish(ceiling_effect_theta_msg);
  ceiling_effect_ratio_pub_.publish(ceiling_effect_ratio_msg);
  ceiling_effect_scale_pub_.publish(ceiling_effect_scale_msg);
  return ceiling_thrust_scale_;
}

double DeltaController::calcCeilingThrustRatio(double dbar, double theta) const
{
  if (dbar > 2.0)
    return 1.0;

  double model_dbar = std::max(ceiling_effect_min_dbar_, dbar);
  double model_theta = std::min(theta, ceiling_effect_max_theta_);
  double coefficient = -6.8407 + 1.6194 * std::sin(model_theta) + 7.2711 * std::cos(model_theta);
  double sqrt_term = 1.0 + coefficient / (8.0 * model_dbar * model_dbar);

  if (sqrt_term < 0.0)
  {
    ROS_WARN_THROTTLE(1.0, "[DeltaController] ceiling effect model sqrt term is negative, use ratio 1.0");
    return 1.0;
  }

  return 0.5 + 0.5 * std::sqrt(sqrt_term);
}

void DeltaController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
  {
    torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

    spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
    Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
    int torque_allocation_matrix_inv_msg_rows = motor_num_;
    torque_allocation_matrix_inv_msg.rows.resize(torque_allocation_matrix_inv_msg_rows);

    if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
      ROS_ERROR("Torque Allocation Matrix overflow");
    for (unsigned int i = 0; i < torque_allocation_matrix_inv_msg_rows; i++)
    {
      torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
      torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
      torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
    }
    torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
  }
}

void DeltaController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  if (use_fc_for_att_control_)
  {
    rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  }
  else
  {
    rpy_gain_msg.motors.at(0).roll_p = 0;
    rpy_gain_msg.motors.at(0).roll_i = 0;
    rpy_gain_msg.motors.at(0).roll_d = 0;
    rpy_gain_msg.motors.at(0).pitch_p = 0;
    rpy_gain_msg.motors.at(0).pitch_i = 0;
    rpy_gain_msg.motors.at(0).pitch_d = 0;
    rpy_gain_msg.motors.at(0).yaw_d = 0;
  }
  rpy_gain_pub_.publish(rpy_gain_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DeltaController, aerial_robot_control::ControlBase);
