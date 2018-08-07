#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <msgs_cctv/PayloadCommand.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <payload_trajectory_cctv/Vec3.h>
#include <geometry_msgs/Vector3.h>


class BearassTracker {

  public:
   BearassTracker();

  private:
    // Global variables
    ros::Subscriber pl_odom_sub;
    ros::Publisher pl_cmd_pub;

    // ROS Service Servers
    ros::ServiceServer setpoint_srv;

    // Fixed Setpoint
    Eigen::Vector3d set_pos;

    // Callbacks
    void pl_odom_cb(const nav_msgs::Odometry::ConstPtr &pl_odom);
};

void BearassTracker::pl_odom_cb(const nav_msgs::Odometry::ConstPtr &pl_odom){

}

//void BearassTracker::setpoint_cb(const geometry_msgs::Vector3::ConstPtr &setpoint_cmd){
//}

 BearassTracker::BearassTracker() {

  ros::NodeHandle priv_nh("~");

  pl_odom_sub = priv_nh.subscribe("payload_odom", 10, &BearassTracker::pl_odom_cb, this, ros::TransportHints().tcpNoDelay());

  pl_cmd_pub = priv_nh.advertise<msgs_cctv::PayloadCommand>("payload_cmd", 10);

  // setpoint_srv = priv_nh.advertiseService("setpoint", &BearassTracker::setpoint_cb, this);
}
