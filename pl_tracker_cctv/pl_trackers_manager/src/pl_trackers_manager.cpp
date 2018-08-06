#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <pl_trackers_manager/Tracker.h>
#include <pl_trackers_manager/Transition.h>
#include <msgs_cctv/PayloadCommand.h>

class PLTrackersManager : public nodelet::Nodelet
{
 public:
  PLTrackersManager(void);
  ~PLTrackersManager(void);

  void onInit(void);

 private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  bool transition_callback(pl_trackers_manager::Transition::Request &req,
                           pl_trackers_manager::Transition::Response &res);

  ros::Subscriber sub_odom_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_status_;
  ros::ServiceServer srv_tracker_;
  pluginlib::ClassLoader<pl_trackers_manager::Tracker> *tracker_loader_;
  pl_trackers_manager::Tracker *active_tracker_;
  std::map<std::string, pl_trackers_manager::Tracker*> tracker_map_;
  msgs_cctv::PayloadCommand::ConstPtr cmd_;
};

PLTrackersManager::PLTrackersManager(void) :
    active_tracker_(NULL)
{
  tracker_loader_ = new pluginlib::ClassLoader<pl_trackers_manager::Tracker>("pl_trackers_manager",
                                                                                   "pl_trackers_manager::Tracker");
}

PLTrackersManager::~PLTrackersManager(void)
{
  std::map<std::string, pl_trackers_manager::Tracker*>::iterator it;
  for(it = tracker_map_.begin(); it != tracker_map_.end(); it++)
  {
    delete it->second;
#if ROS_VERSION_MINIMUM(1,8,0)
    tracker_loader_->unloadLibraryForClass(it->first);
#endif
  }
  delete tracker_loader_;
}

void PLTrackersManager::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  ROS_WARN("--------------------------creating pl trackers manager");
  XmlRpc::XmlRpcValue tracker_list;
  priv_nh.getParam("trackers", tracker_list);
  ROS_ASSERT(tracker_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < tracker_list.size(); i++)
  {
    ROS_WARN("--------------------------loading pl trackers");
    ROS_ASSERT(tracker_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    const std::string tracker_name = static_cast<const std::string>(tracker_list[i]);
    try
    {
#if ROS_VERSION_MINIMUM(1,8,0)
      pl_trackers_manager::Tracker *c = tracker_loader_->createUnmanagedInstance(tracker_name);
#else
      pl_trackers_manager::Tracker *c = tracker_loader_->createClassInstance(tracker_name);
#endif
      c->Initialize(priv_nh);
      tracker_map_.insert(std::make_pair(tracker_name, c));
    }
    catch(pluginlib::LibraryLoadException &e)
    {
      NODELET_ERROR_STREAM("Could not load library for the tracker " << tracker_name << ": " << e.what());
    }
    catch(pluginlib::CreateClassException &e)
    {
      NODELET_ERROR_STREAM("Could not create an instance of the tracker " << tracker_name << ": " << e.what());
    }
  }

  pub_cmd_ = priv_nh.advertise<msgs_cctv::PayloadCommand>("pl_cmd", 10);
  pub_status_ = priv_nh.advertise<quadrotor_msgs::TrackerStatus>("status", 10);

  ROS_WARN("fuck this shit");
  sub_odom_ = priv_nh.subscribe("pl_odom_topic", 10, &PLTrackersManager::odom_callback, this,
                                ros::TransportHints().tcpNoDelay());

  srv_tracker_ = priv_nh.advertiseService("transition", &PLTrackersManager::transition_callback, this);
  ROS_INFO("pl trackers manager should actually exist");
}

void PLTrackersManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
//  ROS_INFO("pl tm got a fucking pl odom");
  std::map<std::string, pl_trackers_manager::Tracker*>::iterator it;
  for(it = tracker_map_.begin(); it != tracker_map_.end(); it++)
  {
    if(it->second == active_tracker_)
    {
      cmd_ = it->second->update(msg);
      if(cmd_ != NULL)
        pub_cmd_.publish(cmd_);

      const quadrotor_msgs::TrackerStatus::Ptr status = it->second->status();
      if(status != NULL)
      {
        status->stamp = ros::Time::now();
        status->tracker = it->first;
        pub_status_.publish(status);
      }
    }
    else
    {
      it->second->update(msg);
    }
  }
}

bool PLTrackersManager::transition_callback(pl_trackers_manager::Transition::Request &req,
                                             pl_trackers_manager::Transition::Response &res)
{
  const std::map<std::string, pl_trackers_manager::Tracker*>::iterator it = tracker_map_.find(req.tracker);
  if(it == tracker_map_.end())
  {
    NODELET_WARN_STREAM("Cannot find tracker " << req.tracker << ", cannot transition");
    return false;
  }
  if(active_tracker_ == it->second)
  {
    NODELET_INFO_STREAM("Tracker " << req.tracker << " already active");
    return true;
  }

  if(!it->second->Activate(cmd_))
  {
    NODELET_WARN_STREAM("Failed to activate tracker " << req.tracker << ", cannot transition");
    return false;
  }

  NODELET_INFO_STREAM("Transitioning to tracker: " << req.tracker);

  if(active_tracker_ != NULL)
    active_tracker_->Deactivate();

  active_tracker_ = it->second;
  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PLTrackersManager, nodelet::Nodelet);
