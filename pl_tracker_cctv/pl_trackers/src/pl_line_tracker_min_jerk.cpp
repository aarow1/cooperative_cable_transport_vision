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

class PLLineTrackerMinJerk : public pl_trackers_manager::Tracker
{
 public:
  PLLineTrackerMinJerk(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd);
  void Deactivate(void);

  const msgs_cctv::PayloadCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

 private:
  void goal_callback(const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg);
  void goal_callback_timed(const quadrotor_msgs::LineTrackerGoalTimed::ConstPtr &msg);

  void gen_trajectory(const Eigen::Vector3d &xi, const Eigen::Vector3d &xf,
                      const Eigen::Vector3d &vi, const Eigen::Vector3d &vf,
                      const Eigen::Vector3d &ai, const Eigen::Vector3d &af,
                      double dt,
                      Eigen::Vector3d coeffs[6]);

  ros::Subscriber sub_goal_, sub_goal_timed_;
  bool pos_set_, goal_set_, goal_reached_;
  double default_v_des_, default_a_des_;
  double v_des_, a_des_;
  bool active_;

  PLInitialConditions ICs_;
  Eigen::Vector3d goal_;
  ros::Time traj_start_;
  double traj_duration_;
  ros::Duration goal_duration_;
  Eigen::Vector3d coeffs_[6];
  bool traj_start_set_;

  double kx_[3], kv_[3];
};

PLLineTrackerMinJerk::PLLineTrackerMinJerk(void)
    : pos_set_(false), goal_set_(false), goal_reached_(true), active_(false),
      traj_start_(ros::Time::now()), traj_start_set_(false)
{
}

void PLLineTrackerMinJerk::Initialize(const ros::NodeHandle &nh)
{
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "line_tracker_min_jerk");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 0.3);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  sub_goal_ = priv_nh.subscribe("goal", 10, &PLLineTrackerMinJerk::goal_callback,
                                this, ros::TransportHints().tcpNoDelay());
  sub_goal_timed_ = priv_nh.subscribe("goal_timed", 10, &PLLineTrackerMinJerk::goal_callback_timed,
                                this, ros::TransportHints().tcpNoDelay());
}

bool PLLineTrackerMinJerk::Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd)
{
  ROS_INFO("pl line tracker min jerk is active");
  // Only allow activation if a goal has been set
  if(goal_set_ && pos_set_)
  {
    active_ = true;
  }
  return active_;
}

void PLLineTrackerMinJerk::Deactivate(void)
{
  ICs_.reset();
  goal_set_ = false;
  active_ = false;
}

// called from odom_callback
const msgs_cctv::PayloadCommand::ConstPtr PLLineTrackerMinJerk::update(
    const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_set_ = true;
  ICs_.set_from_odom(msg);

  const ros::Time t_now = ros::Time::now();

  if(!active_)
    return msgs_cctv::PayloadCommand::Ptr();

  msgs_cctv::PayloadCommand::Ptr cmd(new msgs_cctv::PayloadCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;

  if(goal_set_)
  {
    if(!traj_start_set_)
      traj_start_ = t_now;

    bool duration_set = false;
    traj_duration_ = 0.5f;

    // TODO: This should probably be after traj_duration_ is determined
    if(goal_duration_.toSec() > traj_duration_){
      traj_duration_ = goal_duration_.toSec();
      duration_set = true;
    }

    // Min-Jerk trajectory
    const double total_dist = (goal_ - ICs_.pos()).norm();
    const Eigen::Vector3d dir = (goal_ - ICs_.pos()) / total_dist;
    const double vel_proj = (ICs_.vel()).dot(dir);

    const double t_ramp = (v_des_ - vel_proj) / a_des_;

    const double distance_to_v_des =
        vel_proj * t_ramp + 0.5f * a_des_ * t_ramp * t_ramp;
    const double distance_v_des_to_stop = 0.5f * v_des_ * v_des_ / a_des_;

    const double ramping_distance = distance_to_v_des + distance_v_des_to_stop;

    if(!duration_set) // If duration is not set by the goal callback
    {
      if(total_dist > ramping_distance)
      {
        double t = (v_des_ - vel_proj) / a_des_ // Ramp up
          + (total_dist - ramping_distance) / v_des_ // Constant velocity
          + v_des_ / a_des_; // Ramp down

        traj_duration_ = std::max(traj_duration_, t);
      }
      else
      {
        // In this case, v_des_ is not reached. Assume bang bang acceleration.

        double vo = vel_proj;
        double distance_to_stop = 0.5f * vo * vo / a_des_;

        double t_dir; // The time required for the component along dir
        if(vo > 0.0f && total_dist < distance_to_stop)
        {
          // Currently traveling towards the goal and need to overshoot

          t_dir = vo / a_des_ +
            std::sqrt(2.0f) *
            std::sqrt(vo * vo - 2.0f * a_des_ * total_dist) / a_des_;
        }
        else
        {
          // Ramp up to a velocity towards the goal before ramping down

          t_dir = -vo / a_des_ +
            std::sqrt(2.0f) *
            std::sqrt(vo * vo + 2.0f * a_des_ * total_dist) / a_des_;
        }
        traj_duration_ = std::max(traj_duration_, t_dir);

        // The velocity component orthogonal to dir
        double v_ortho = (ICs_.vel() - dir * vo).norm();
        double t_non_dir =
          v_ortho / a_des_ // Ramp to zero velocity
          + std::sqrt(2.0f) * v_ortho / a_des_; // Get back to the dir line

        traj_duration_ = std::max(traj_duration_, t_non_dir);
      }
    }


    gen_trajectory(ICs_.pos(), goal_, ICs_.vel(), Eigen::Vector3d::Zero(),
                   ICs_.acc(), Eigen::Vector3d::Zero(),
                   traj_duration_, coeffs_);

    goal_set_ = false;
  }
  else if(goal_reached_)
  {
    cmd->pos.position.x = goal_(0);
    cmd->pos.position.y = goal_(1);
    cmd->pos.position.z = goal_(2);
    cmd->vel.linear.x = 0;
    cmd->vel.linear.y = 0;
    cmd->vel.linear.z = 0;
    cmd->acc.linear.x = 0;
    cmd->acc.linear.y = 0;
    cmd->acc.linear.z = 0;
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

  Eigen::Vector3d x(ICs_.pos()), v(Eigen::Vector3d::Zero()),
      a(Eigen::Vector3d::Zero()), j(Eigen::Vector3d::Zero());

  const double traj_time = (t_now - traj_start_).toSec();

  if(traj_time >= traj_duration_) // Reached goal
  {
    ROS_DEBUG_THROTTLE(1, "Reached goal");
    j = Eigen::Vector3d::Zero();
    a = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    x = goal_;
    goal_reached_ = true;
  }
  else if(traj_time >= 0)
  {
    double t = traj_time / traj_duration_, t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

    x = coeffs_[0] + t * coeffs_[1] + t2 * coeffs_[2] + t3 * coeffs_[3] +
        t4 * coeffs_[4] + t5 * coeffs_[5];
    v = coeffs_[1] + 2 * t * coeffs_[2] + 3 * t2 * coeffs_[3] +
        4 * t3 * coeffs_[4] + 5 * t4 * coeffs_[5];
    a = 2 * coeffs_[2] + 6 * t * coeffs_[3] + 12 * t2 * coeffs_[4] +
        20 * t3 * coeffs_[5];
    j = 6 * coeffs_[3] + 24 * t * coeffs_[4] + 60 * t2 * coeffs_[5];

    // Scale based on the trajectory duration
    v = v / traj_duration_;
    a = a / (traj_duration_ * traj_duration_);
    j = j / (traj_duration_ * traj_duration_ * traj_duration_);
  }
  else // (traj_time < 0) can happen with LineTrackerGoalTimed
    ROS_INFO_THROTTLE(1, "Trajectory hasn't started yet");

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

void PLLineTrackerMinJerk::goal_callback(
    const quadrotor_msgs::LineTrackerGoal::ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;
  goal_duration_ = ros::Duration(0); // Clear the stored value of goal_duration_, will be replaced by heurisitic
  traj_start_set_ = false;

  if (msg->relative)
  {
    goal_ += ICs_.pos();
    ROS_INFO("line_tracker_min_jerk using relative command");
  }

  if(msg->v_des > 0.0)
    v_des_ = msg->v_des;
  else
    v_des_ = default_v_des_;

  if(msg->a_des > 0.0)
    a_des_ = msg->a_des;
  else
    a_des_ = default_a_des_;

  ROS_DEBUG("line_tracker_min_jerk using v_des = %2.2f m/s and a_des = %2.2f m/s^2", v_des_, a_des_);

  goal_set_ = true;
  goal_reached_ = false;
}

// TODO(Kartik): Maybe merge the two goal callbacks since by default t_start and
// duration would be zero if not set
void PLLineTrackerMinJerk::goal_callback_timed(
    const quadrotor_msgs::LineTrackerGoalTimed::ConstPtr &msg)
{
  goal_(0) = msg->x;
  goal_(1) = msg->y;
  goal_(2) = msg->z;
  goal_duration_ = msg->duration;
  traj_start_ = msg->t_start;
  traj_start_set_ = true;
  ROS_DEBUG("Starting trajectory in %2.2f seconds.", (traj_start_ - ros::Time::now()).toSec());

  if (msg->relative)
  {
    goal_ += ICs_.pos();
    ROS_INFO("line_tracker_min_jerk using relative command");
  }

  goal_set_ = true;
  goal_reached_ = false;
}

void PLLineTrackerMinJerk::gen_trajectory(
    const Eigen::Vector3d &xi, const Eigen::Vector3d &xf,
    const Eigen::Vector3d &vi, const Eigen::Vector3d &vf,
    const Eigen::Vector3d &ai, const Eigen::Vector3d &af,
    double dt,
    Eigen::Vector3d coeffs[6])
{
  // We can use a dt of 1 to ensure that our system will be numerically conditioned.
  // For more information, see line_tracker_min_jerk_numerical_issues.m

  Eigen::Matrix<double, 6, 6> Ainv;
  Ainv <<   1,    0,    0,    0,    0,    0,
            0,    0,    1,    0,    0,    0,
            0,    0,    0,    0,  0.5,    0,
          -10,   10,   -6,   -4, -1.5,  0.5,
           15,  -15,    8,    7,  1.5,   -1,
           -6,    6,   -3,   -3, -0.5,  0.5;

  Eigen::Matrix<double, 6, 3> b;
  b << xi.transpose(), xf.transpose(),
      dt * vi.transpose(), dt * vf.transpose(),
      dt * dt * ai.transpose(), dt * dt * af.transpose();

  Eigen::Matrix<double, 6, 3> x;
  x = Ainv * b;
  for(int i = 0; i < 6; i++)
  {
    coeffs[i] = x.row(i).transpose();
  }
}

const quadrotor_msgs::TrackerStatus::Ptr PLLineTrackerMinJerk::status()
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
PLUGINLIB_EXPORT_CLASS(PLLineTrackerMinJerk, pl_trackers_manager::Tracker)
