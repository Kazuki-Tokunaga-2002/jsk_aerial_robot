#include <delta/control/delta_wrench_allocation.h>
#include <nlopt.hpp>
#include <cmath>

using namespace aerial_robot_control;

namespace
{
double angleDiff(double from, double to)
{
  return std::atan2(std::sin(from - to), std::cos(from - to));
}
}  // namespace

double nonlinearWrenchAllocationMinObjective(const std::vector<double>& x, std::vector<double>& grad, void* ptr)
{
  // 0 ~ motor_num : lambda
  // motor_num ~ 2 * motor_num : gimbal roll (phi)

  DeltaController* controller = reinterpret_cast<DeltaController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  controller->setNLOptIterations(controller->getNLOptIterations() + 1);

  double ret = 0;
  int motor_num = robot_model_for_control->getRotorNum();
  const auto& prev_lambda = controller->getLambdaAll();
  const auto& prev_phi = controller->getTargetGimbalAngles();
  const auto& nominal_phi = controller->getNLOptPhiNominal();
  const double lambda_weight = controller->getNLOptLambdaWeight();
  const double delta_lambda_weight = controller->getNLOptDeltaLambdaWeight();
  const double phi_nominal_weight = controller->getNLOptPhiNominalWeight();
  const double delta_phi_weight = controller->getNLOptDeltaPhiWeight();
  const double lambda_balance_weight = controller->getNLOptLambdaBalanceWeight();
  double lambda_mean = 0.0;

  if (lambda_balance_weight > 0.0)
  {
    for (int i = 0; i < motor_num; i++)
      lambda_mean += x.at(i);
    lambda_mean /= motor_num;
  }

  /* compute result */
  for (int i = 0; i < motor_num; i++)
  {
    const double lambda = x.at(i);
    const double phi = x.at(i + motor_num);
    const double d_lambda = lambda - prev_lambda.at(i);
    const double d_phi = angleDiff(phi, prev_phi.at(i));
    const double d_nominal_phi = angleDiff(phi, nominal_phi.at(i));
    const double d_balance_lambda = lambda - lambda_mean;

    ret += lambda_weight * lambda * lambda;
    ret += delta_lambda_weight * d_lambda * d_lambda;
    ret += delta_phi_weight * d_phi * d_phi;
    ret += phi_nominal_weight * d_nominal_phi * d_nominal_phi;
    ret += lambda_balance_weight * d_balance_lambda * d_balance_lambda;
  }

  if (grad.empty())
    return ret;

  /* make gradient */
  for (int i = 0; i < motor_num; i++)
  {
    const double lambda = x.at(i);
    const double phi = x.at(i + motor_num);

    grad.at(i) = 2.0 * lambda_weight * lambda;
    grad.at(i) += 2.0 * delta_lambda_weight * (lambda - prev_lambda.at(i));
    grad.at(i) += 2.0 * lambda_balance_weight * (lambda - lambda_mean);

    grad.at(i + motor_num) = 2.0 * delta_phi_weight * angleDiff(phi, prev_phi.at(i));
    grad.at(i + motor_num) += 2.0 * phi_nominal_weight * angleDiff(phi, nominal_phi.at(i));
  }
  return ret;
}

void nonlinearWrenchAllocationEqConstraints(unsigned m, double* result, unsigned n, const double* x, double* grad,
                                            void* ptr)
{
  // 0 ~ 6 : Q * lambda - target_wrench_cog = 0

  DeltaController* controller = reinterpret_cast<DeltaController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  int motor_num = robot_model_for_control->getRotorNum();

  std::vector<Eigen::Matrix3d> links_rotation_from_cog =
      robot_model_for_control->getLinksRotationFromCog<Eigen::Matrix3d>();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      robot_model_for_control->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<double> rotor_tilt = controller->getRotorTilt();

  Eigen::VectorXd target_acc_cog = controller->getTargetAccCog();
  double mass = robot_model_for_control->getMass();
  Eigen::Matrix3d inertia = robot_model_for_control->getInertia<Eigen::Matrix3d>();
  double m_f_rate = robot_model_for_control->getMFRate();
  std::map<int, int> rotor_direction = robot_model_for_control->getRotorDirection();

  Eigen::VectorXd target_wrench_cog = target_acc_cog;
  target_wrench_cog.head(3) = mass * target_wrench_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_cog.tail(3);

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(motor_num);
  for (int i = 0; i < motor_num; i++)
    lambda(i) = x[i];

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, motor_num);
  for (int i = 0; i < motor_num; i++)
  {
    Eigen::Vector3d u_Li;
    u_Li << sin(rotor_tilt.at(i)), -sin(x[i + motor_num]) * cos(rotor_tilt.at(i)),
        cos(x[i + motor_num]) * cos(rotor_tilt.at(i));

    // cog_u_Li = cog_R_Li * u_Li
    Q.block(0, i, 3, 1) = links_rotation_from_cog.at(i) * u_Li;
    Q.block(3, i, 3, 1) = (aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                           m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) *
                          links_rotation_from_cog.at(i) * u_Li;
  }

  Eigen::VectorXd wrench_diff = Q * lambda - target_wrench_cog;
  for (int i = 0; i < m; i++)
    result[i] = wrench_diff(i);

  if (grad == NULL)
    return;

  /* make gradient */
  // thrust part. dwrench_dlambda = Q */
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < motor_num; j++)
    {
      grad[i * n + j] = Q(i, j);
    }
  }

  /* gimbal part */
  Eigen::MatrixXd df_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  Eigen::MatrixXd dtau_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  for (int i = 0; i < motor_num; i++)
  {
    Eigen::Vector3d du_Li_dphi;
    du_Li_dphi << 0, -cos(x[i + motor_num]) * cos(rotor_tilt.at(i)), -sin(x[i + motor_num]) * cos(rotor_tilt.at(i));

    df_dphi.block(0, i, 3, 1) = links_rotation_from_cog.at(i) * du_Li_dphi * x[i];
    dtau_dphi.block(0, i, 3, 1) = (aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) *
                                  links_rotation_from_cog.at(i) * du_Li_dphi * x[i];
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < motor_num; j++)
    {
      grad[i * n + (j + motor_num)] = df_dphi(i, j);
      grad[(i + 3) * n + (j + motor_num)] = dtau_dphi(i, j);
    }
  }
}
