#ifndef CONTROL_CCTV_H
#define CONTROL_CCTV_H

#include <string>
#include <Eigen/Geometry>
#include <ros/ros.h>

class ControlCCTV
{
 public:
  ControlCCTV();

//  void setMass(const double mass);
//  void setGravity(const double g);
//  void setPosition(const Eigen::Vector3d &position);
//  void setVelocity(const Eigen::Vector3d &velocity);
//  void setMaxIntegral(const double max_integral);
//  void setMaxIntegralBody(const double max_integral_b);
//  void setCurrentOrientation(const Eigen::Quaterniond &current_orientation);
//  void resetIntegrals();
//  void setMaxTiltAngle(const double max_tilt_angle);

  // System Setters
  void set_m_0    (const double m_0);
  void set_J_0    (const Eigen::Matrix3d J_0);
  void set_m_i    (const double m_i);
  void set_J_i    (const Eigen::Matrix3d J_i);
  void set_l_i    (const double l_i);
  void set_rho    (const Eigen::Matrix<double, 3, 3> rho);  //TODO make this dynamic size
  void set_g      (const double g);
  void set_idx    (const int idx);
  void set_n_bots (const int n_bots);

  // State Setters
  void set_pos_0  (const Eigen::Vector3d &pos_0);
  void set_vel_0  (const Eigen::Vector3d &vel_0);
  void set_R_0    (const Eigen::Matrix3d &R_0);
  void set_Omega_0(const Eigen::Vector3d &Omega_0);
  void set_R_i    (const Eigen::Matrix3d &R_i);
  void set_Omega_i(const Eigen::Vector3d &Omega_i);
  void set_q_i    (const Eigen::Vector3d &q_i);
  void set_q_i_dot(const Eigen::Vector3d &q_i_dot);
  void set_w_i    (const Eigen::Vector3d &w_i);

  void calculateControl(const Eigen::Vector3d &des_pos_0,
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
                        const double &k_w);


  // Getters
  const Eigen::Vector3d     &getComputedForce();
  const Eigen::Quaterniond  &getComputedOrientation();
  const Eigen::Vector3d     &getComputedAngularVelocity();
  const Eigen::Vector3d     &get_u_i_parallel();
  const Eigen::Vector3d     &get_u_i_perpendicular();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:

  // System Definition
  double m_0_;              // payload mass [kg]
  Eigen::Matrix3d J_0_;     // payload inertia (3x3)
  double m_i_;              // quadrotor mass [kg]
  Eigen::Matrix3d J_i_;     // quadrotor inertia (3x3)
  int n_bots_;              // number of robots, TODO make this a param
  int idx_;                 // quadrotor index number
  double l_i_;              // length of my cable (may change to an (nx1) vector)
  Eigen::Matrix<double, 3, 3> rho_; // attachment points of all cables (3xn matrix), CONSTANT, TODO make this sized by param?
  double g_;

  // States
  Eigen::Vector3d pos_0_;   // payload position
  Eigen::Vector3d vel_0_;   // payload velocity
  Eigen::Matrix3d R_0_;     // payload orientation in world frame
  Eigen::Vector3d Omega_0_; // payload angular velocity in body frame
  Eigen::Matrix3d R_i_;     // my quadrotor orientation in world frame
  Eigen::Vector3d Omega_i_; // my quadrotor angular velocity in quadrotor frame
  Eigen::Vector3d q_i_;     // my cable direction, unit vector, world frame
  Eigen::Vector3d q_i_dot_; // cable direction derivative. TODO: this is fishy...
  Eigen::Vector3d w_i_;     // my cable angular velocity, world?? frame
  Eigen::Matrix<double, 6, 9> P_;   // Control matrix, CONSTANT
  Eigen::Matrix<double, 9, 6> pseudo_P_;

  // Errors
  // TODO: should these be global?
//  Eigen::Vector3d e_pos_0_;   // payload position error
//  Eigen::Vector3d e_vel_0_;   // payload velocity error
//  Eigen::Vector3d e_R_0_;     // payload orientation in world frame error
//  Eigen::Vector3d e_Omega_0_; // payload angular velocity in body frame error

  // Gains
  // TODO: decide if these should be params. Maybe dynamic reconfigure
//  double k_pos_0_;          // payload position gain
//  double k_vel_0_;          // payload velocity gain
//  double k_R_0_;            // payload orientation gain
//  double k_Omega_0_;        // payload angular velocity gain
//  double k_q;               // cable direction gain
//  double k_w;               // cable angular velocity gain

  // Payload forces
//  Eigen::Vector3d F_0_des_;   // Desired overall payload force (calculated)
//  Eigen::Vector3d M_0_des_;   // Desired overall payload moment (calculated)
//  Eigen::Matrix<double, 9, 1> mu_des_;  // Desired virtual controls for all robots
//  Eigen::Vector3d mu_i_des_;  // Desired virtual control for this robot
  Eigen::Vector3d q_i_des_;   // Desired cable direction for this robot
  Eigen::Vector3d w_i_des;
//  Eigen::Vector3d mu_i_;      // Virtual Control for this robot
//  Eigen::Vector3d a_i_;
//  Eigen::Vector3d u_i_parallel_;

  // Prior state tracking (for numerical derivatives)
  ros::Time t_last_control;

  // Outputs of the controller
  Eigen::Vector3d force_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d angular_velocity_;

  // debugging outputs of controller
  Eigen::Vector3d u_i_parallel;
  Eigen::Vector3d u_i_perpendicular;
  //  Eigen::Vector3d pos_int_;
  //  Eigen::Vector3d pos_int_b_;

  //  Did not replace these yet
  //  double max_pos_int_;
  //  double max_pos_int_b_;
  //  double cos_max_tilt_angle_;
};

#endif
