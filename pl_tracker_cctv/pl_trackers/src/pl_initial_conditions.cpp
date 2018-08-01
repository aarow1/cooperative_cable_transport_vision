/**
 * In most cases we would want to have consecutive trajectories to smoothly
 * transition from one to another, so we need to keep the desired command
 * continuous. This class stores the last command so as to use it as an initial
 * condition for the next trajectory leading to a smooth desired command.
 * Previously, we used the current odom of the robot to set the initial
 * condition and if the robot had some tracking error, it would lead to a jump
 * in the desired, which we want to avoid.
 */

#include <tf/transform_datatypes.h>
#include <pl_initial_conditions.h>
#include <msgs_cctv/PayloadCommand.h>

PLInitialConditions::PLInitialConditions() :
  pos_(Eigen::Vector3d::Zero()),
  vel_(Eigen::Vector3d::Zero()),
  acc_(Eigen::Vector3d::Zero()),
  orr_(Eigen::Quaterniond::Identity()),
  omg_(Eigen::Vector3d::Zero()),
  alf_(Eigen::Vector3d::Zero()),
  cmd_valid_(false)
{
}

void PLInitialConditions::set_from_cmd(
    const msgs_cctv::PayloadCommand::ConstPtr &msg)
{
  if (msg == NULL)
  {
    ROS_WARN("Null PositionCommand recieved. Not setting initial condition.");
    return;
  }

  pos_ = Eigen::Vector3d(msg->pos.position.x,
                         msg->pos.position.y,
                         msg->pos.position.z);
  vel_ = Eigen::Vector3d(msg->vel.linear.x,
                         msg->vel.linear.y,
                         msg->vel.linear.z);
  acc_ = Eigen::Vector3d(msg->acc.linear.x,
                         msg->acc.linear.y,
                         msg->acc.linear.z);

  orr_ = Eigen::Quaterniond(msg->pos.orientation.x,
                            msg->pos.orientation.y,
                            msg->pos.orientation.z,
                            msg->pos.orientation.w);
  omg_ = Eigen::Vector3d(msg->vel.angular.x,
                         msg->vel.angular.y,
                         msg->vel.angular.z);
  alf_ = Eigen::Vector3d(msg->acc.angular.x,
                         msg->acc.angular.y,
                         msg->acc.angular.z);

  cmd_valid_ = true;
}

void PLInitialConditions::set_from_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!cmd_valid_)
  {
    pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
    vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
    acc_ = Eigen::Vector3d::Zero();

    orr_ = Eigen::Quaterniond(msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z,
                              msg->pose.pose.orientation.w);
    omg_ = Eigen::Vector3d(msg->twist.twist.angular.x,
                           msg->twist.twist.angular.y,
                           msg->twist.twist.angular.z);
    alf_ = Eigen::Vector3d::Zero();
  }
}

void PLInitialConditions::reset()
{
  cmd_valid_ = false;
}
