#ifndef CONTROL_CCTV_H
#define CONTROL_CCTV_H

#include <string>
#include <Eigen/Geometry>
#include <ros/ros.h>

using namespace Eigen;

class ControlCCTV
{
 public:
  ControlCCTV();


//  void setMass(const double mass);
//  void setGravity(const double g);
//  void setPosition(const Vector3d &position);
//  void setVelocity(const Vector3d &velocity);
//  void setMaxIntegral(const double max_integral);
//  void setMaxIntegralBody(const double max_integral_b);
//  void setCurrentOrientation(const Quaterniond &current_orientation);
//  void resetIntegrals();
//  void setMaxTiltAngle(const double max_tilt_angle);

  // System Setters
  void set_m_0    (const double m_0);
  void set_J_0    (const Matrix3d J_0);
  void set_m_i    (const double m_i);
  void set_J_i    (const Matrix3d J_i);
  void set_l_i    (const double l_i);
  void set_rho    (const Matrix<double, 3, 3> rho);  //TODO make this dynamic size
  void set_g      (const double g);
  void set_idx    (const int idx);
  void set_n_bots (const int n_bots);

  // State Setters
  void set_pos_0  (const Vector3d &pos_0);
  void set_vel_0  (const Vector3d &vel_0);
  void set_R_0    (const Matrix3d &R_0);
  void set_Omega_0(const Vector3d &Omega_0);
  void set_R_i    (const Matrix3d &R_i);
  void set_Omega_i(const Vector3d &Omega_i);
  void set_q_i    (const Vector3d &q_i);
  void set_q_i_dot(const Vector3d &q_i_dot);
  void set_w_i    (const Vector3d &w_i);

  // Control Calculation
  void calculateControl(const Vector3d &des_pos_0,
                        const Vector3d &des_vel_0,
                        const Vector3d &des_acc_0,
                        const Matrix3d &des_R_0,
                        const Vector3d &des_Omega_0,
                        const Vector3d &des_alpha_0,
                        const double &des_yaw_i,
                        const Vector3d &k_pos_0,
                        const Vector3d &ki_pos_0,
                        const double &k_vel_0,
                        const double &k_R_0,
                        const double &k_Omega_0,
                        const double &k_q,
                        const double &k_w);


  // Getters
  const Vector3d     &getComputedForce();
  const Quaterniond  &getComputedOrientation();
  const Vector3d     &getComputedAngularVelocity();
  const Vector3d     &get_u_i();
  const Vector3d     &get_u_i_prl();
  const Vector3d     &get_u_i_prp();

  // debugging outputs of controller TODO make these private
  Vector3d e_pos_0  = Vector3d::Zero();
  Vector3d e_vel_0  = Vector3d::Zero();
  Vector3d e_R_0    = Vector3d::Zero();
  Vector3d e_Omega_0  = Vector3d::Zero();
  Vector3d e_q_i    = Vector3d::Zero();
  Vector3d e_w_i    = Vector3d::Zero();

  Vector3d pos_0_int = Vector3d::Zero();
  double max_pos_0_int = 1.0;

  Vector3d F_0_des  = Vector3d::Zero();
  Vector3d M_0_des  = Vector3d::Zero();
  Vector3d q_i_des  = -Vector3d::UnitZ();
  Vector3d u_i_     = Vector3d::Zero();
  Vector3d u_i_prl  = Vector3d::Zero();
  Vector3d u_i_prp  = Vector3d::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


private:

  // System Definition
  double    m_0_;            // payload mass [kg]
  Matrix3d  J_0_;            // payload inertia (3x3)
  double    m_i_;            // quadrotor mass [kg]
  Matrix3d  J_i_;            // quadrotor inertia (3x3)
  int       n_bots_;         // number of robots, TODO make this a param
  int       idx_;            // quadrotor index number
  double    l_i_;            // length of my cable (may change to an (nx1) vector)
  Matrix<double, 3, 3> rho_; // attachment points of all cables (3xn matrix), CONSTANT, TODO make this sized by param?
  double    g_;              // Gravity constant

  // States
  Vector3d pos_0_;            // payload position
  Vector3d vel_0_;            // payload velocity
  Matrix3d R_0_;              // payload orientation in world frame
  Vector3d Omega_0_;          // payload angular velocity in body frame
  Matrix3d R_i_;              // my quadrotor orientation in world frame
  Vector3d Omega_i_;          // my quadrotor angular velocity in quadrotor frame
  Vector3d q_i_;              // my cable direction, unit vector, world frame
  Vector3d q_i_dot_;          // cable direction derivative. TODO: this is fishy...
  Vector3d w_i_;              // my cable angular velocity, world?? frame
  Matrix<double, 6, 9> P_;    // Control matrix, CONSTANT
  Matrix<double, 9, 6> P_inv_;// Psuedo Inverse of P

  // Outputs of the controller
  Vector3d force_;
  Quaterniond orientation_;
  Vector3d angular_velocity_;


  // Minimum norm of a vector where it can be reliably normalized
  const double eps = 1e-6f;

  // Integral accumulators
  //  Vector3d pos_int_;
  //  Vector3d pos_int_b_;

  //  Did not replace these yet
  //  double max_pos_int_;
  //  double max_pos_int_b_;
  //  double cos_max_tilt_angle_;
};

#endif
