#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Geometry>
#include <msgs_cctv/PayloadTrajCommand.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

// Global variables
ros::Publisher pl_odom_pub;
ros::Publisher pl_viz_pub;
ros::Subscriber input_wrench_sub;

// Payload state
Eigen::Vector3d     pos;
Eigen::Vector3d     vel;
Eigen::Quaterniond  orr;
Eigen::Quaterniond  omg;

// Function prototypes
void pub_marker_viz(nav_msgs::Odometry pl_msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  pl_odom_pub = n.advertise<nav_msgs::Odometry>("pl_pose", 1000);
  pl_viz_pub  = n.advertise<visualization_msgs::Marker>("pl_viz", 1000);

  input_wrench_sub = n.subscribe("external_wrench", 100, input_wrench_cb);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    const bool MOVE = 0;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "simulator";

    double t = ros::Time::now().toSec();
    double omega_x = 0.1;
    double omega_y = 0.2;
    double a_x = 0.5;
    double a_y = 0.5;

    if (MOVE){
      odom_msg.pose.pose.position.x = a_x*sin(omega_x * t);
      odom_msg.pose.pose.position.y = a_y*sin(omega_y * t);
      odom_msg.pose.pose.position.z = 0.0;

      odom_msg.twist.twist.linear.x = a_x*omega_x*cos(omega_x*t);
      odom_msg.twist.twist.linear.y = a_y*omega_x*cos(omega_y*t);
      odom_msg.twist.twist.linear.z = 0;

      double eul_z = 0.0; // fmod(0.1*t, 2*M_PI); // yaw
      double eul_y = 0.0; // 0.5*sin(0.5*t); // pitch
      double eul_x = 0; // roll

      Eigen::Quaternionf pl_orientation;
      pl_orientation =
          Eigen::AngleAxisf(eul_z, Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(eul_y, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(eul_x, Eigen::Vector3f::UnitX());

      ROS_INFO("eul_z %2.2f", eul_z);

      odom_msg.pose.pose.orientation.x = pl_orientation.x();
      odom_msg.pose.pose.orientation.y = pl_orientation.y();
      odom_msg.pose.pose.orientation.z = pl_orientation.z();
      odom_msg.pose.pose.orientation.w = pl_orientation.w();
    } else {
      odom_msg.pose.pose.position.x = 0.0;
      odom_msg.pose.pose.position.y = 0.0;
      odom_msg.pose.pose.position.z = 0.0;

      odom_msg.twist.twist.linear.x = 0.0;
      odom_msg.twist.twist.linear.y = 0.0;
      odom_msg.twist.twist.linear.z = 0.0;

      odom_msg.pose.pose.orientation.x = 0.0;
      odom_msg.pose.pose.orientation.y = 0.0;
      odom_msg.pose.pose.orientation.z = 0.0;
      odom_msg.pose.pose.orientation.w = 1.0;
    }

    pl_odom_pub.publish(odom_msg);
    pub_marker_viz(odom_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

void input_wrench_cb(geometry_msgs::WrenchStamped msg){
  const Eigen::Vector3d input_force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
  const Eigen::Vector3d input_moment(msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z);
}

void pub_marker_viz(nav_msgs::Odometry pl_msg){
  visualization_msgs::Marker marker;

  marker.header = pl_msg.header;
  marker.pose = pl_msg.pose.pose;

  // marker setup
  marker.ns = "payload_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 1.0;
  marker.scale.y = 2.0;
  marker.scale.z = 0.02;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  pl_viz_pub.publish(marker);
}