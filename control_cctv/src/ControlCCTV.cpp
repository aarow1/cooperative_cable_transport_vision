#include "control_cctv/ControlCCTV.h"
#include "control_cctv/vio_utils.h"

#define DEBUG 0

// TODO: Remove CLAMP as macro
#define CLAMP(x,min,max) ((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x)

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

  const double eps = 1e-6f;

  const Eigen::Vector3d e_pos_0     = pos_0_ - des_pos_0;
  const Eigen::Vector3d e_vel_0     = vel_0_ - des_vel_0;
  const Eigen::Vector3d e_R_0       = 0.5 * VIOUtil::vee( (des_R_0.transpose() * R_0_)
                                  - (R_0_.transpose()    * des_R_0));
  const Eigen::Vector3d e_Omega_0   = Omega_0_ - (R_0_.transpose() * des_R_0 * des_Omega_0);

  Eigen::Vector3d test_v3;
//  ROS_WARN_THROTTLE(1, "EIGEN TEST: [%2.2f, %2.2f, %2.2f]", test_v3(0), test_v3(1), test_v3(2));
  ROS_WARN_THROTTLE(1, "================================================================");
//  ROS_WARN_THROTTLE(1, "pos_0: [%2.2f, %2.2f, %2.2f]", pos_0_(0), pos_0_(1), pos_0_(2));
//  ROS_WARN_THROTTLE(1, "des_pos_0: [%2.2f, %2.2f, %2.2f]", des_pos_0(0), des_pos_0(1), des_pos_0(2));
  ROS_WARN_THROTTLE(1, "e_pos_0: [%2.2f, %2.2f, %2.2f]", e_pos_0(0), e_pos_0(1), e_pos_0(2));
  ROS_WARN_THROTTLE(1, "k_pos_0: %2.2f", k_pos_0);
  ROS_WARN_THROTTLE(1, "m_0_: %2.2f", m_0_);
  ROS_WARN_THROTTLE(1, "e_vel_0: [%2.2f, %2.2f, %2.2f]", e_vel_0(0), e_vel_0(1), e_vel_0(2));
  ROS_WARN_THROTTLE(1, "e_R_0: [%2.2f, %2.2f, %2.2f]", e_R_0(0), e_R_0(1), e_R_0(2));
  ROS_WARN_THROTTLE(1, "e_Omega_0: [%2.2f, %2.2f, %2.2f]", e_Omega_0(0), e_Omega_0(1), e_Omega_0(2));


  // Calculate desired payload force and moment
  // TODO: add integral terms
  const Eigen::Vector3d F_0_des_ = m_0_ * (   (-1.0*k_pos_0 * e_pos_0)
                                           +  (-1.0*k_vel_0 * e_vel_0)
                                           +  des_acc_0);
                                           //gravity compensation
//                                           + (g_ * Eigen::Vector3d::UnitZ()));  // changed from minut to plus because z is up

 ROS_WARN_THROTTLE(1, "F_0_des_: [%2.4f, %2.4f, %2.4f]", F_0_des_(0), F_0_des_(1), F_0_des_(2));

  const Eigen::Vector3d M_0_des_ = -k_R_0 * e_R_0
      - k_Omega_0 * e_Omega_0
      + VIOUtil::getSkew( R_0_.transpose() * des_R_0 * des_Omega_0 )
      * (J_0_ * R_0_.transpose() * des_R_0 * des_Omega_0)
      + J_0_ * R_0_.transpose() * des_R_0  * des_alpha_0;

  ROS_WARN_THROTTLE(1, "M_0_des_: [%2.2f, %2.2f, %2.2f]", M_0_des_(0), M_0_des_(1), M_0_des_(2));



  // Calculated desired virtual controls for all robots
  Eigen::MatrixXd diagonal_R_0;
  diagonal_R_0.resize(3*n_bots_, 3*n_bots_);
  diagonal_R_0.setZero();
  for (int i = 0; i < n_bots_; i++){
    diagonal_R_0.block<3,3>(3*i, 3*i) = R_0_;
  }
//  ROS_INFO_STREAM_COND(DEBUG, "diagonal_R_0 size is (" << diagonal_R_0.rows() << ", " <<diagonal_R_0.cols());
//  ROS_INFO_STREAM_COND(DEBUG, "diagonal_R_0_ is\n" << diagonal_R_0);


  Eigen::Matrix<double, 6, 1> control_0_des;
  control_0_des.block<3,1>(0,0) = R_0_.transpose() * F_0_des_;
  control_0_des.block<3,1>(3,0) = M_0_des_;


  const Eigen::Matrix<double, 9,1> mu_des = diagonal_R_0 * pseudo_P_ * control_0_des;

  // extract my virtual control input
  const Eigen::Vector3d mu_i_des = mu_des.block<3,1>(3*idx_, 0);
  const Eigen::Vector3d mu_i = q_i_ * q_i_.transpose() * mu_i_des;

  ROS_WARN_THROTTLE(1, "q_i_: [%2.2f, %2.2f, %2.2f]", q_i_(0), q_i_(1), q_i_(2));
  ROS_WARN_THROTTLE(1, "mu_i_des: [%2.2f, %2.2f, %2.2f]", mu_i_des(0), mu_i_des(1), mu_i_des(2));
  ROS_WARN_THROTTLE(1, "mu_i: [%2.2f, %2.2f, %2.2f]", mu_i(0), mu_i(1), mu_i(2));

  const Eigen::Vector3d q_i_des_last = q_i_des_;
  if(mu_i_des.norm() > eps)
    q_i_des_ = -mu_i_des.normalized();
  else
    q_i_des_ = -Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d debug_cross = q_i_des_.cross(q_i_des_.cross(q_i_des_last));

  // numerical derivative of q_i_des
  Eigen::Vector3d q_i_des_dot_direction = (q_i_des_.cross(q_i_des_.cross(q_i_des_last))); //.normalized(); // TODO make this analytic
  if (q_i_des_dot_direction.norm() > eps)
    q_i_des_dot_direction.normalize();
  else
    q_i_des_dot_direction = Eigen::Vector3d::Zero();

  const double q_i_des_dot_magnitude = (std::acos(
                                          CLAMP(
                                            q_i_des_.dot(q_i_des_last), -1.0, 1.0))) / dt;
  const Eigen::Vector3d q_i_des_dot = q_i_des_dot_direction * q_i_des_dot_magnitude;

  ROS_WARN_THROTTLE(1, "q_i_des_: [%2.2f, %2.2f, %2.2f]", q_i_des_(0), q_i_des_(1), q_i_des_(2));
  ROS_WARN_THROTTLE(1, "q_i_des_dot_direction: [%2.2f, %2.2f, %2.2f]", q_i_des_dot_direction(0), q_i_des_dot_direction(1), q_i_des_dot_direction(2));
  ROS_WARN_THROTTLE(1, "q_i_des_dot_magnitude: %2.2f", q_i_des_dot_magnitude);
  ROS_WARN_THROTTLE(1, "q_i_des_dot: [%2.2f, %2.2f, %2.2f]", q_i_des_dot(0), q_i_des_dot(1), q_i_des_dot(2));

  // feed forward quadrotor acceleration
  const Eigen::Vector3d a_i = (des_acc_0)
      + (g_ * Eigen::Vector3d::UnitZ())
      + (R_0_ * VIOUtil::getSkew(Omega_0_) * VIOUtil::getSkew(Omega_0_) * rho_.block<3,1>(0,idx_))
      - (R_0_ * VIOUtil::getSkew(rho_.block<3,1>(0,idx_)) * des_alpha_0);

  ROS_WARN_THROTTLE(1, "controller a_i = [%2.2f, %2.2f, %2.2f]", a_i(0), a_i(1), a_i(2));

  // Parallel component of control
  u_i_parallel = mu_i
      + (m_i_ * l_i_ * (w_i_.norm() * w_i_.norm()) * q_i_)
      + (m_i_ * q_i_ * q_i_.transpose() * a_i);


  ROS_WARN_THROTTLE(1, "w_i_: [%2.2f, %2.2f, %2.2f]", w_i_(0), w_i_(1), w_i_(2));
  ROS_WARN_THROTTLE(1, "u_i_parallel: [%2.2f, %2.2f, %2.2f]", u_i_parallel(0), u_i_parallel(1), u_i_parallel(2));

  const Eigen::Vector3d w_i_des_last = w_i_des;
  w_i_des = q_i_des_.cross(q_i_des_dot);   // <eq 26.5>
  const Eigen::Vector3d w_i_des_dot = (w_i_des - w_i_des_last) / dt;
  ROS_WARN_THROTTLE(1, "w_i_des_dot: [%2.2f, %2.2f, %2.2f]", w_i_des_dot(0), w_i_des_dot(1), w_i_des_dot(2));
  ROS_WARN_THROTTLE(1, "w_i_des: [%2.2f, %2.2f, %2.2f]", w_i_des(0), w_i_des(1), w_i_des(2));


  const Eigen::Matrix3d q_i_hat = VIOUtil::getSkew(q_i_);
  const Eigen::Matrix3d q_i_hat_2 = q_i_hat * q_i_hat;
  ROS_WARN_THROTTLE(1, "q_i_hat: [%2.2f, %2.2f, %2.2f]", q_i_hat(0,0), q_i_hat(0,1), q_i_hat(0,2));
  ROS_WARN_THROTTLE(1, "q_i_hat: [%2.2f, %2.2f, %2.2f]", q_i_hat(1,0), q_i_hat(1,1), q_i_hat(1,2));
  ROS_WARN_THROTTLE(1, "q_i_hat: [%2.2f, %2.2f, %2.2f]", q_i_hat(2,0), q_i_hat(2,1), q_i_hat(2,2));

  const Eigen::Vector3d e_q_i = q_i_des_.cross(q_i_);
  const Eigen::Vector3d e_w_i = w_i_ + q_i_hat_2 * w_i_des;
  ROS_WARN_THROTTLE(1, "e_q_i: [%2.2f, %2.2f, %2.2f]", e_q_i(0), e_q_i(1), e_q_i(2));
  ROS_WARN_THROTTLE(1, "k_q: %2.2f", k_q);
  ROS_WARN_THROTTLE(1, "e_w_i: [%2.2f, %2.2f, %2.2f]", e_w_i(0), e_w_i(1), e_w_i(2));
  ROS_WARN_THROTTLE(1, "q_i_dot_: [%2.2f, %2.2f, %2.2f]", q_i_dot_(0), q_i_dot_(1), q_i_dot_(2));

  // Perpendicular component of control <eq 27>
  u_i_perpendicular = (m_i_*l_i_*q_i_hat) *
        (-1.0*k_q*e_q_i)
        - (k_w*e_w_i)
        - (q_i_.dot(w_i_des) * q_i_dot_)  // TODO: q_i_dot is fishy
        - (q_i_hat_2 *w_i_des_dot)
        - (m_i_ * q_i_hat_2 * a_i); //TODO: add delta term here, equation 27

  const Eigen::Vector3d u_i_ = u_i_parallel + u_i_perpendicular;
  ROS_WARN_THROTTLE(1, "m_i_: %2.2f", m_i_);
  ROS_WARN_THROTTLE(1, "l_i_: %2.2f", l_i_);

  const Eigen::Matrix3d perp_db = (m_i_ * l_i_ * q_i_hat);
  ROS_WARN_THROTTLE(1, "perp_db: [%2.2f, %2.2f, %2.2f]", perp_db(0,0), perp_db(0,1), perp_db(0,2));
  ROS_WARN_THROTTLE(1, "perp_db: [%2.2f, %2.2f, %2.2f]", perp_db(1,0), perp_db(1,1), perp_db(1,2));
  ROS_WARN_THROTTLE(1, "perp_db: [%2.2f, %2.2f, %2.2f]", perp_db(2,0), perp_db(2,1), perp_db(2,2));

  ROS_WARN_THROTTLE(1, "u_i_parallel: [%2.2f, %2.2f, %2.2f]", u_i_parallel(0), u_i_parallel(1), u_i_parallel(2));
  ROS_WARN_THROTTLE(1, "u_i_perpendicular: [%2.2f, %2.2f, %2.2f]", u_i_perpendicular(0), u_i_perpendicular(1), u_i_perpendicular(2));
  ROS_WARN_THROTTLE(1, "u_i_: [%2.2f, %2.2f, %2.2f]", u_i_(0), u_i_(1), u_i_(2));
  force_ = u_i_;
  for(int i = 0; i<3; i++){
    if (std::isnan(force_(i))) ROS_ERROR("FORCE HAS A NAN");
  }

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
  ROS_ERROR_COND(std::isnan(orientation_.x()), "ORIENTATION x IS NAN");
  ROS_ERROR_COND(std::isnan(orientation_.y()), "ORIENTATION y IS NAN");
  ROS_ERROR_COND(std::isnan(orientation_.z()), "ORIENTATION z IS NAN");
  ROS_ERROR_COND(std::isnan(orientation_.w()), "ORIENTATION w IS NAN");
  ROS_WARN_THROTTLE(1, "orientation_: [%2.2f, %2.2f, %2.2f, %2.2f]", orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w());

  // Numerically calculate angular velocity (R_c - R_c_last) / dt
  Eigen::Vector3d Omega_1_0 = VIOUtil::LogSO3(R_c_last.transpose()*R_c) * (1/dt);
  Eigen::Vector3d Omega_1_1 = R_c.transpose() * R_c_last * Omega_1_0;
  angular_velocity_ = Omega_1_1;
  ROS_ERROR_COND(std::isnan(angular_velocity_(0)), "ANGULAR VELOCITY X IS NAN");
  ROS_ERROR_COND(std::isnan(angular_velocity_(1)), "ANGULAR VELOCITY Y IS NAN");
  ROS_ERROR_COND(std::isnan(angular_velocity_(2)), "ANGULAR VELOCITY Z IS NAN");
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
const Eigen::Vector3d     &ControlCCTV::get_u_i_parallel(){
  return u_i_parallel;
}
const Eigen::Vector3d     &ControlCCTV::get_u_i_perpendicular(){
  return u_i_perpendicular;
}
