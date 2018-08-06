#include <ros/ros.h>
#include <pl_trackers_manager/Tracker.h>
#include <quadrotor_msgs/TrackerStatus.h>
#include <msgs_cctv/PayloadCommand.h>
#include <Eigen/Geometry>

class PLHoverTracker : public pl_trackers_manager::Tracker
{
 public:
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd);
  void Deactivate(void);

  const msgs_cctv::PayloadCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  const quadrotor_msgs::TrackerStatus::Ptr status();

private:
  bool pose_set = false;
  Eigen::Vector3d hover_pos;
  Eigen::Quaterniond hover_orr;
};

void PLHoverTracker::Initialize(const ros::NodeHandle &nh)
{
  hover_pos = Eigen::Vector3d::Zero();
  hover_orr = Eigen::Quaterniond::Identity();
}

bool PLHoverTracker::Activate(const msgs_cctv::PayloadCommand::ConstPtr &cmd)
{
  ROS_INFO("pl hover tracker is active");
  pose_set = false;
  return true;
}

void PLHoverTracker::Deactivate(void)
{
}

const msgs_cctv::PayloadCommand::ConstPtr PLHoverTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(!pose_set){
    hover_pos(0) = msg->pose.pose.position.x;
    hover_pos(1) = msg->pose.pose.position.y;
    hover_pos(2) = msg->pose.pose.position.z;
    // hover_orr.x() = msg->pose.pose.orientation.x;
    // hover_orr.y() = msg->pose.pose.orientation.y;
    // hover_orr.z() = msg->pose.pose.orientation.z;
    // hover_orr.w() = msg->pose.pose.orientation.w;
    hover_orr = Eigen::Quaterniond::Identity();
    pose_set = true;
    ROS_INFO("pl hover tracker set hover pos to %2.2f, %2.2f, %2.2f", hover_pos(0), hover_pos(1), hover_pos(2));
  }

  ROS_INFO("hover tracker update");
  msgs_cctv::PayloadCommand::Ptr cmd(new msgs_cctv::PayloadCommand);
  cmd->header.stamp = ros::Time::now();
  cmd->header.frame_id = msg->header.frame_id;

  cmd->pos.position.x = hover_pos(0);
  cmd->pos.position.y = hover_pos(1);
  cmd->pos.position.z = hover_pos(2);

  cmd->pos.orientation.x = hover_orr.x();
  cmd->pos.orientation.y = hover_orr.y();
  cmd->pos.orientation.z = hover_orr.z();
  cmd->pos.orientation.w = hover_orr.w();

  // Everything else inits to zero

  return cmd;
}

const quadrotor_msgs::TrackerStatus::Ptr PLHoverTracker::status()
{
  quadrotor_msgs::TrackerStatus::Ptr msg(new quadrotor_msgs::TrackerStatus);
  msg->status = quadrotor_msgs::TrackerStatus::SUCCEEDED;
  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PLHoverTracker, pl_trackers_manager::Tracker)
