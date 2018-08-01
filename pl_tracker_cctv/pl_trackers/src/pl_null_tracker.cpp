#include <ros/ros.h>
#include <pl_trackers_manager/Tracker.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <msgs_cctv/PayloadCommand.h>

class PLNullTracker : public pl_trackers_manager::Tracker
{
 public:
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd);
  void Deactivate(void);

  const msgs_cctv::PayloadCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();
};

void PLNullTracker::Initialize(const ros::NodeHandle &nh)
{
}

bool PLNullTracker::Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd)
{
  ROS_INFO_STREAM("Null tracker is active");
  return true;
}

void PLNullTracker::Deactivate(void)
{
  ROS_INFO_STREAM("Null tracker is deactivated");
}

const msgs_cctv::PayloadCommand::ConstPtr PLNullTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Return a null message (will not publish the position command)
  msgs_cctv::PayloadCommand::Ptr cmd;


  return msgs_cctv::PayloadCommand::Ptr();
}

const quadrotor_msgs::TrackerStatus::Ptr PLNullTracker::status()
{
  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::SUCCEEDED;
  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PLNullTracker, pl_trackers_manager::Tracker)
