#ifndef PL_TRACKERS_INITIAL_CONDITIONS_H
#define PL_TRACKERS_INITIAL_CONDITIONS_H

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <msgs_cctv/PayloadCommand.h>

class PLInitialConditions
{
 public:
  PLInitialConditions();
   
  void set_from_cmd(const msgs_cctv::PayloadCommand::ConstPtr &msg);
  void set_from_odom(const nav_msgs::Odometry::ConstPtr &msg);
  Eigen::Vector3d pos() const { return pos_; }
  Eigen::Vector3d vel() const { return vel_; }
  Eigen::Vector3d acc() const { return acc_; }

  Eigen::Quaterniond orr() const {return orr_; }
  Eigen::Vector3d omg() const { return omg_; }
  Eigen::Vector3d alf() const { return alf_; }


  void reset();

 private:
  // linear
  Eigen::Vector3d pos_, vel_, acc_;

  // angular
  Eigen::Quaterniond orr_; // orientation
  Eigen::Vector3d omg_, alf_; // angular velocity, acceleration

  bool cmd_valid_;
};

#endif // STD_TRACKERS_INITIAL_CONDITIONS_H
