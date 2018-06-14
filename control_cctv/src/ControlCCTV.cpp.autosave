#include "control_cctv/ControlCCTV.h"
#include "control_cctv/vio_utils.h"

#define DEBUG 1

ControlCCTV::ControlCCTV()
  : m_0_(0.5),
    g_(9.81),
    n_bots_(3),
    idx_(2),
    R_0_(Eigen::Matrix3d::Identity())
{
  if (idx_ >= n_bots_) ROS_ERROR("Robot index (%i) is larger than total number of robots (%i)", idx_, n_bots_);

  // Retrieve attachment point params for all robots
//  for (int i=0; i<n_bots_; i++){
//    ROS_INFO("loading rho %i", i);
//    std::string param_name = "attachment/rho_" + std::to_string(i);
//    if(!priv_nh_.getParam(param_name + "/x", rho_(0, i)))
//      ROS_ERROR("Couldn't find param: %s/x", param_name.c_str());
//    if(!priv_nh_.getParam(param_name + "/y", rho_(1, i)))
//      ROS_ERROR("Couldn't find param: %s/y", param_name.c_str());
//    if(!priv_nh_.getParam(param_name + "/z", rho_(2, i)))
//      ROS_ERROR("Couldn't find param: %s/z", param_name.c_str());
//  }
}

// System Setters
void ControlCCTV::set_m_0    (const double m_0){
  m_0_ = m_0;
}
void ControlCCTV::set_J_0    (const Eigen::Matrix3d J_0){
  J_0_ = J_0;
}
void ControlCCTV::set_m_i    (const double m_i){
  m_i_ = m_i;
}
void ControlCCTV::set_J_i    (const Eigen::Matrix3d J_i){
  J_i_ = J_i;
}
void ControlCCTV::set_l_i    (const double l_i){
  l_i_ = l_i;
}
void ControlCCTV::set_rho    (const Eigen::Matrix<double, 3, 3> rho){
  rho_ = rho;

  // add attachments to control matrix P
  for(int i = 0; i < n_bots_; i++){
    P_.block<3,3>(0, 3*i) = Eigen::Matrix3d::Identity();
    P_.block<3,3>(3, 3*i) = VIOUtil::getSkew(rho_.block<3,1>(0,i));
  }

  // P must have full rank for the payload to be controllable
  typedef Eigen::Matrix<double, 6, 9> Matrixd6x9;
  Eigen::FullPivLU<Matrixd6x9> lu(P_);
  ROS_ERROR_COND(lu.rank() < P_.rows(), "P is not full rank");
  pseudo_P_ = P_.transpose() * (P_*P_.transpose()).inverse();
}
void ControlCCTV::set_g      (const double g){
  g_ = g;
}
void ControlCCTV::set_idx    (const int idx){
  idx_ = idx;
}
void ControlCCTV::set_n_bots (const int n_bots){
  n_bots_ = n_bots;
}

// State Setters
void ControlCCTV::set_pos_0  (const Eigen::Vector3d &pos_0){
  pos_0_ = pos_0;
}
void ControlCCTV::set_vel_0  (const Eigen::Vector3d &vel_0){
  vel_0_ = vel_0;
}
void ControlCCTV::set_R_0    (const Eigen::Matrix3d &R_0){
  R_0_ = R_0;
}
void ControlCCTV::set_Omega_0(const Eigen::Vector3d &Omega_0){
  Omega_0_ = Omega_0;
}
void ControlCCTV::set_R_i    (const Eigen::Matrix3d &R_i){
  R_i_ = R_i;
}
void ControlCCTV::set_Omega_i(const Eigen::Vector3d &Omega_i){
  Omega_i_ = Omega_i;
}
void ControlCCTV::set_q_i    (const Eigen::Vector3d &q_i){
  q_i_ = q_i;
}
void ControlCCTV::set_q_i_dot(const Eigen::Vector3d &q_i_dot){
  q_i_dot_ = q_i_dot;
}
void ControlCCTV::set_w_i    (const Eigen::Vector3d &w_i){
  w_i_ = w_i;
}


void ControlCCTV::calculateControl(const Eigen::Vector3d &des_pos_0,
                      const Eigen::Vector3d &des_vel_0,
                      const Eigen::Vector3d &des_acc_0,
                      const Eigen::Matrix3d &des_R_0,
                      const Eigen::Vector3d &des_Omega_0,
                      const Eigen::Vector3d &des_alpha_0,
                      const double &des_yaw_i,
                      const double &k_pos_0,
                      const double &k_vel_0,
                      const double &k_R_0,
                      const double &k_Omega_0,
                      const double &k_q,
                      const double &k_w)
{
  double dt = (ros::Time::now() - t_last_control).toSec();
  t_last_control = ros::Time::now();

  ROS_DEBUG_COND(DEBUG, "line 52");
  const Eigen::Vector3d e_pos_0     = pos_0_ - des_pos_0;
  const Eigen::Vector3d e_vel_0     = vel_0_ - des_vel_0;
  const Eigen::Vector3d e_R_0       = 0.5 * VIOUtil::vee( (des_R_0.transpose() * R_0_)
                                  - (R_0_.transpose()    * des_R_0));
  const Eigen::Vector3d e_Omega_0   = Omega_0_ - (R_0_.transpose() * des_R_0 * des_Omega_0);

  ROS_DEBUG_COND(DEBUG, "line 59");

  // Calculate desired payload force and moment
  // TODO: add integral terms
  const Eigen::Vector3d F_0_des_ = m_0_ * (-k_pos_0_ * e_pos_0
                                           -k_vel_0_ * e_vel_0
                                           + des_acc_0
                                           + g_ * Eigen::Vector3d::UnitZ());  // made this plus because I like z up

  ROS_DEBUG_COND(DEBUG, "line 68");

  const Eigen::Vector3d M_0_des_ = -k_R_0_ * e_R_0
      - k_Omega_0_ * e_Omega_0
      + VIOUtil::getSkew( R_0_.transpose() * des_R_0 * des_Omega_0 )
      * (J_0_ * R_0_.transpose() * des_R_0 * des_Omega_0)
      + J_0_ * R_0_.transpose() * des_R_0  * des_alpha_0;

  ROS_DEBUG_COND(DEBUG, "line 76");

  // Calculated desired virtual controls for all robots
  Eigen::MatrixXd diagonal_R_0;
  diagonal_R_0.resize(3*n_bots_, 3*n_bots_);
  diagonal_R_0.setZero();
  for (int i = 0; i < n_bots_; i++){
    diagonal_R_0.block<3,3>(3*i, 3*i) = R_0_;
  }
  ROS_INFO_STREAM_COND(DEBUG, "diagonal_R_0 size is (" << diagonal_R_0.rows() << ", " <<diagonal_R_0.cols());
  ROS_INFO_STREAM_COND(DEBUG, "diagonal_R_0_ is\n" << diagonal_R_0);

  ROS_DEBUG_COND(DEBUG, "line 85");

  Eigen::Matrix<double, 6, 1> control_0_des;
  control_0_des.block<3,1>(0,0) = R_0_.transpose() * F_0_des_;
  control_0_des.block<3,1>(3,0) = M_0_des_;

  ROS_DEBUG_COND(DEBUG, "line 95");

  const Eigen::Matrix<double, 9,1> mu_des = diagonal_R_0 * pseudo_P_ * control_0_des;

  // extract my virtual control input
  const Eigen::Vector3d mu_i_des = mu_des.block<3,1>(3*idx_, 0);
  const Eigen::Vector3d mu_i = q_i_ * q_i_.transpose() * mu_i_des;

  ROS_DEBUG_COND(DEBUG, "line 103");

  const Eigen::Vector3d q_i_des_last = q_i_des_;
  q_i_des_ = -mu_i_des.normalized();
  const Eigen::Vector3d q_i_des_dot_direction = (q_i_des_.cross(q_i_des_.cross(q_i_des_last))).normalized(); // TODO make this analytic
  const double q_i_des_dot_magnitude = std::acos(q_i_des_.dot(q_i_des_last)) / dt;
  const Eigen::Vector3d q_i_des_dot = q_i_des_dot_direction * q_i_des_dot_magnitude;

  ROS_DEBUG_COND(DEBUG, "line 110");

  const Eigen::Vector3d a_i = des_acc_0 + g_ * Eigen::Vector3d::UnitZ()
      + R_0_ * VIOUtil::getSkew(Omega_0_) * VIOUtil::getSkew(Omega_0_) * rho_.block<3,1>(0,idx_)
      - R_0_ * VIOUtil::getSkew(rho_.block<3,1>(0,idx_)) * des_alpha_0;

  ROS_DEBUG_COND(DEBUG, "line 113");

  // Parallel component of control
  const Eigen::Vector3d u_i_parallel = mu_i + (m_i_ * l_i_ * (w_i_.norm() * w_i_.norm()) * q_i_)
      + m_i_ * q_i_ * q_i_.transpose() * a_i;

  ROS_DEBUG_COND(DEBUG, "line 120");

  const Eigen::Vector3d w_i_des_last = w_i_des;
  w_i_des = q_i_des_.cross(q_i_des_dot);   // <eq 26.5>
  const Eigen::Vector3d w_i_des_dot = (w_i_des - w_i_des_last) / dt;

  ROS_DEBUG_COND(DEBUG, "line 125");

  const Eigen::Matrix3d q_i_hat = VIOUtil::getSkew(q_i_);
  const Eigen::Matrix3d q_i_hat_2 = q_i_hat * q_i_hat;

  const Eigen::Vector3d e_q_i = q_i_des_.cross(q_i_);
  const Eigen::Vector3d e_w_i = w_i_ + q_i_hat_2 * w_i_des;

  ROS_DEBUG_COND(DEBUG, "line 133");

  // Perpendicular component of control <eq 27>
  const Eigen::Vector3d u_i_perpendicular = m_i_ * l_i_ * q_i_hat * (
        -k_q*e_q_i - k_w*e_w_i
        - (q_i_.dot(w_i_des) * q_i_dot_)  // TODO: q_i_dot is fishy
        - q_i_hat_2 *w_i_des_dot)
      - m_i_ * q_i_hat_2 * a_i; //TODO: add delta term here, equation 27

  ROS_DEBUG_COND(DEBUG, "through calculate_control");

  const Eigen::Vector3d u_i_ = u_i_parallel + u_i_perpendicular;
  force_ = u_i_;

  // Taken directly from SO3Control, skipped angular velocity
  const Eigen::Vector3d b2d(-std::sin(des_yaw_i), std::cos(des_yaw_i), 0);

  Eigen::Vector3d b1c, b2c, b3c;
  if(u_i_.norm() > 1e-6f)
    b3c.noalias() = u_i_.normalized();
  else
    b3c.noalias() = Eigen::Vector3d::UnitZ();

  b1c.noalias() = b2d.cross(b3c).normalized();
  b2c.noalias() = b3c.cross(b1c).normalized();

  Eigen::Matrix3d R_c;
  R_c << b1c, b2c, b3c;
  Eigen::Matrix3d R_c_last = orientation_.normalized().toRotationMatrix();
  orientation_ = Eigen::Quaterniond(R_c);

  // Numerically calculate angular velocity (R_c - R_c_last) / dt
  Eigen::Vector3d Omega_1_0 = VIOUtil::LogSO3(R_c_last.transpose()*R_c) * (1/dt);
  Eigen::Vector3d Omega_1_1 = R_c.transpose() * R_c_last * Omega_1_0;
  angular_velocity_ = Omega_1_1;
}

const Eigen::Vector3d     &ControlCCTV::getComputedForce(){
  return force_;
}
const Eigen::Quaterniond  &ControlCCTV::getComputedOrientation(){
  return orientation_;
}
const Eigen::Vector3d     &ControlCCTV::getComputedAngularVelocity(){
  return angular_velocity_;
}
