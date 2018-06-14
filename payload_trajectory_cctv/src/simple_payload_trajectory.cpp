#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <msgs_cctv/PayloadTrajCommand.h>
#include <visualization_msgs/Marker.h>

// Global variables
ros::Publisher pl_pose_pub;
ros::Publisher pl_viz_pub;

// Function prototypes
void pub_marker_viz(msgs_cctv::PayloadTrajCommand pl_msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  pl_pose_pub = n.advertise<msgs_cctv::PayloadTrajCommand>("pl_pose", 1000);
  pl_viz_pub  = n.advertise<visualization_msgs::Marker>("pl_viz", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    msgs_cctv::PayloadTrajCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "global";

    double t = ros::Time::now().toSec();
    double omega_x = 1;
    double omega_y = 2;

    msg.pose.position.x = sin(omega_x * t);
    msg.pose.position.y = sin(omega_y * t);
    msg.pose.position.z = 0.0;

    double eul_z = fmod(t, 2*M_PI); // yaw
    double eul_y = 0.5*sin(0.5*t); // pitch
    double eul_x = 0; // roll

    Eigen::Quaternionf pl_orientation;
    pl_orientation =
        Eigen::AngleAxisf(eul_z, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(eul_y, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(eul_x, Eigen::Vector3f::UnitX());

    ROS_INFO("eul_z %2.2f", eul_z);

    msg.pose.orientation.x = pl_orientation.x();
    msg.pose.orientation.y = pl_orientation.y();
    msg.pose.orientation.z = pl_orientation.z();
    msg.pose.orientation.w = pl_orientation.w();

    pl_pose_pub.publish(msg);
    pub_marker_viz(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

void pub_marker_viz(msgs_cctv::PayloadTrajCommand pl_msg){
  visualization_msgs::Marker marker;

  marker.header = pl_msg.header;
  marker.pose = pl_msg.pose;

  // marker setup
  marker.ns = "payload_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 1.0;
  marker.scale.y = 2.0;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  pl_viz_pub.publish(marker);
}
