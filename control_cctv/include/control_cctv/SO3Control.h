#ifndef SO3_CONTROL_H
#define SO3_CONTROL_H

#include <Eigen/Geometry>

class SO3Control
{
 public:
  SO3Control();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d &position);
  void setVelocity(const Eigen::Vector3d &velocity);
  void setMaxIntegral(const double max_integral);
  void setMaxIntegralBody(const double max_integral_b);
  void setCurrentOrientation(const Eigen::Quaterniond &current_orientation);
  void resetIntegrals();
  void setMaxTiltAngle(const double max_tilt_angle);

  void calculateControl(const Eigen::Vector3d &des_pos,
                        const Eigen::Vector3d &des_vel,
                        const Eigen::Vector3d &des_acc,
                        const Eigen::Vector3d &des_jerk,
                        const double des_yaw,
                        const double des_yaw_dot,
                        const Eigen::Vector3d &kx,
                        const Eigen::Vector3d &kv,
                        const Eigen::Vector3d &ki,
                        const Eigen::Vector3d &ki_b);

  const Eigen::Vector3d &getComputedForce();
  const Eigen::Quaterniond &getComputedOrientation();
  const Eigen::Vector3d &getComputedAngularVelocity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Inputs for the controller
  double mass_;
  double g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double max_pos_int_;
  double max_pos_int_b_;
  Eigen::Quaterniond current_orientation_;
  double cos_max_tilt_angle_;

  // Outputs of the controller
  Eigen::Vector3d force_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d pos_int_;
  Eigen::Vector3d pos_int_b_;
};

#endif
