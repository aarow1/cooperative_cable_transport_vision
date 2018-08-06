#include <iostream>
#include <ros/ros.h>
#include <pl_trackers_manager/Tracker.h>
#include <quadrotor_msgs/LineTrackerGoal.h>
#include <quadrotor_msgs/LineTrackerGoalTimed.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <pl_initial_conditions.h>
#include <msgs_cctv/PayloadCommand.h>

// Order of the polynomial, the highest power is 11, there are 12 coefficients
#define POLY_ORDER 12

class PLLineTracker : public pl_trackers_manager::Tracker
{
 public:
  PLLineTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd);
  void Deactivate(void);

  const msgs_cctv::PayloadCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:
  void goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg);
  Eigen::Matrix<double, 12, 1> get_t_vector(ros::Time t_now, int derivative);
//  void goal_callback_timed(const quadrotor_msgs::LineTrackerGoalTimed::ConstPtr &msg);

  // Generate a trajectory from a starting position to a goal position
  // Assumes homogenous boundaries for all derivatives

  PLInitialConditions ICs_;
  ros::Subscriber sub_goal_, sub_goal_timed_;

  bool start_set;
  bool goal_set_;
  bool traj_set_;

  // bool traj_time_set;
  bool goal_reached_;
  bool pos_set_; // Has the tracker received any odometry

  double default_v_des_, default_a_des_;
  double v_des_, a_des_;
  bool active_;

  Eigen::Matrix<double, 3, 12> x_coeffs_;
  Eigen::Matrix<double, 3, 12> v_coeffs_;
  Eigen::Matrix<double, 3, 12> a_coeffs_;

  Eigen::Vector3d start_, goal_;
  ros::Time traj_start_time_;
  double traj_duration_;
//  ros::Duration goal_duration_;
//  Eigen::Vector3d coeffs_[POLY_ORDER];
//  bool traj_start_set_;

  double kx_[3], kv_[3];
};

PLLineTracker::PLLineTracker(void)
    : pos_set_(false), goal_set_(false), goal_reached_(true), active_(false),
      traj_start_time_(ros::Time::now())
{
}

void PLLineTracker::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "line_tracker");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.3);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &PLLineTracker::goal_callback,
                                this, ros::TransportHints().tcpNoDelay());
}

bool PLLineTracker::Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd)
{
  ROS_INFO("Activating Line Tracker");

  // Only allow activation if a goal has been set and odometry has been received
  if(goal_set_ && pos_set_)
  {
    active_ = true;
  }
  return active_;
}

void PLLineTracker::Deactivate(void)
{
  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

// This is called for every payload odom callback, independent of whether this tracker is active
const msgs_cctv::PayloadCommand::ConstPtr PLLineTracker::update(
    const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  if(!active_) return msgs_cctv::PayloadCommand::Ptr();

  const ros::Time t_now = ros::Time::now();

  msgs_cctv::PayloadCommand::Ptr cmd(new msgs_cctv::PayloadCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;

  if(!traj_set_)
  {
    // Precalculated coeffs for a normalized trajectory with homogeneous boundaries
    Eigen::Matrix<double, 1, 12> base_coeffs_;
    base_coeffs_ << 0, 0, 0, 0, 0, 0, 462, -1980, 3465, -3080, 1386, -252;

    // Modifiers used to evaluate derivatives
    Eigen::Matrix<double, 3, 12> coeff_modifiers_;
    coeff_modifiers_ <<
                        1, 1, 1, 1,  1,  1,  1,  1,  1,  1,  1,   1,
                        0, 1, 2, 3,  4,  5,  6,  7,  8,  9, 10,  11,
                        0, 0, 2, 6, 12, 20, 30, 42, 56, 72, 90, 110;

    start_ = ICs_.pos();
    Eigen::Vector3d delta_pos = goal_ - start_;
    ROS_INFO_STREAM("start is " << start_);
    ROS_INFO_STREAM("goal is " << goal_);

    for(int i = 0; i<12; i++){
      x_coeffs_.block<3,1>(0,i) = delta_pos * base_coeffs_(0, i) * coeff_modifiers_(0, i);
      v_coeffs_.block<3,1>(0,i) = delta_pos * base_coeffs_(0, i) * coeff_modifiers_(1, i);
      a_coeffs_.block<3,1>(0,i) = delta_pos * base_coeffs_(0, i) * coeff_modifiers_(2, i);
    }

    ROS_INFO_STREAM("x coeffs are " << x_coeffs_);

    traj_set_ = true;
    traj_start_time_ = t_now;
    traj_duration_ = delta_pos.norm() / v_des_;
    ROS_INFO("line mad e a traj, duration is %2.2f", traj_duration_);
  }

  Eigen::Vector3d x(ICs_.pos());
  Eigen::Vector3d v(Eigen::Vector3d::Zero());
  Eigen::Vector3d a(Eigen::Vector3d::Zero());

  const double traj_time = (t_now - traj_start_time_).toSec();

  if(traj_time >= traj_duration_) // Reached goal
  {
    ROS_INFO("line tracker Reached goal");
    a = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    x = goal_;
    goal_reached_ = true;
  }
  else if(traj_time >= 0) // Actually doing the trajectory
  {
    Eigen::Matrix<double, 12, 1> tx_vec = get_t_vector(t_now, 0);
    Eigen::Matrix<double, 12, 1> tv_vec = get_t_vector(t_now, 1);
    Eigen::Matrix<double, 12, 1> ta_vec = get_t_vector(t_now, 2);

    ROS_INFO("traj_time is %2.2f, duration is %2.2f", traj_time, traj_duration_);
    ROS_INFO_STREAM("tx_vec is " << tx_vec);
    ROS_INFO_STREAM("x_coeffs_ are " << x_coeffs_);
    ROS_INFO_STREAM("x is " << x);
    x = start_ + x_coeffs_ * tx_vec;
    v = v_coeffs_ * tv_vec;
    a = a_coeffs_ * ta_vec;

  }
  else // (traj_time < 0) can happen with LineTrackerGoalTimed
    ROS_INFO("Trajectory hasn't started yet");

  cmd->pos.position.x = x(0), cmd->pos.position.y = x(1), cmd->pos.position.z = x(2);
  cmd->vel.linear.x = v(0), cmd->vel.linear.y = v(1), cmd->vel.linear.z = v(2);
  cmd->acc.linear.x = a(0), cmd->acc.linear.y = a(1), cmd->acc.linear.z = a(2);
  cmd->pos.orientation.x = 0;
  cmd->pos.orientation.y = 0;
  cmd->pos.orientation.z = 0;
  cmd->pos.orientation.w = 1;
  cmd->vel.angular.x = 0;
  cmd->vel.angular.y = 0;
  cmd->vel.angular.z = 0;
  cmd->acc.angular.x = 0;
  cmd->acc.angular.y = 0;
  cmd->acc.angular.z = 0;

  ICs_.set_from_cmd(cmd);
  return cmd;
}

void PLLineTracker::goal_callback(
    const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg)
{
  ROS_INFO("Line tracker got new goal");
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;
//  goal_duration_ = ros::Duration(0); // Clear the stored value of goal_duration_, will be replaced by heurisitic
//  traj_start_set_ = false;

  if (msg->relative)
  {
    goal_ += ICs_.pos();
    ROS_INFO("line_tracker using relative command");
  }

  if(msg->v_des > 0.0)
    v_des_ = msg->v_des;
  else
    v_des_ = default_v_des_;

  if(msg->a_des > 0.0)
    a_des_ = msg->a_des;
  else
    a_des_ = default_a_des_;

  ROS_DEBUG("line_tracker using v_des = %2.2f m/s and a_des = %2.2f m/s^2", v_des_, a_des_);

  goal_set_ = true;
  goal_reached_ = false;
  traj_set_ = false;
}

// Calculates correct powers of t and scales based on traj_duration
Eigen::Matrix<double, 12, 1> PLLineTracker::get_t_vector(ros::Time t_now, int derivative){
  // First normalize t based on traj_start_time and traj_duration
  double t_normalized = ((t_now - traj_start_time_)).toSec()/traj_duration_;
  ROS_INFO("t_normalized is %2.2f", t_normalized);

  Eigen::Matrix<double, 12, 1> t_vector;
  t_vector.setZero(); // Since the for loop starts at derivative, all entries before that stay zero

  for(int i = derivative; i<12; i++){
    t_vector(i) = (pow(t_normalized, i-derivative)) / (pow(traj_duration_, derivative));
  }
  return t_vector;
}

const quadrotor_msgs::TrackerStatus::Ptr PLLineTracker::status()
{
  if(!active_)
    return quadrotor_msgs::TrackerStatus::Ptr();

  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);

  msg->status = goal_reached_ ?
          static_cast<uint8_t>(quadrotor_msgs::TrackerStatus::SUCCEEDED) :
          static_cast<uint8_t>(quadrotor_msgs::TrackerStatus::ACTIVE);

  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PLLineTracker, pl_trackers_manager::Tracker)
