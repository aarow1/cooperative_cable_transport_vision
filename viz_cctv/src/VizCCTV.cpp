#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Geometry>
#include <msgs_cctv/PayloadTrajCommand.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

// Global variables
ros::Subscriber payload_odom_sub, uav_odom_sub, cable_q_sub, cable_q_dot_sub, cable_w_sub, payload_vel_sub, u_i_sub, u_i_parallel_sub, u_i_perpendicular_sub;
ros::Publisher  payload_odom_pub, uav_odom_pub, cable_q_pub, cable_q_dot_pub, cable_w_pub, payload_vel_pub, u_i_pub, u_i_parallel_pub, u_i_perpendicular_pub;

Eigen::Vector3f uav_position, payload_position;
geometry_msgs::Point uav_position_point, payload_position_point;
double cable_length;
std::string fixed_frame_id;

// Function prototypes
void payload_odom_callback( nav_msgs::Odometry msg);
void uav_odom_callback(     nav_msgs::Odometry msg);
void cable_q_callback(      geometry_msgs::Vector3Stamped msg);
void cable_q_dot_callback(  geometry_msgs::Vector3Stamped msg);
void cable_w_callback(      geometry_msgs::Vector3Stamped msg);
void payload_vel_callback(  geometry_msgs::Vector3Stamped msg);
void u_i_callback(          geometry_msgs::Vector3Stamped msg);
void u_i_parallel_callback( geometry_msgs::Vector3Stamped msg);
void u_i_perpendicular_callback(  geometry_msgs::Vector3Stamped msg);

visualization_msgs::Marker arrow_marker(std_msgs::Header msg_header);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viz_cctv");

  ros::NodeHandle n("~");

  // Subscribers
  payload_odom_sub  = n.subscribe("payload_odom_topic", 1000, &payload_odom_callback);
  uav_odom_sub      = n.subscribe("uav_odom_topic",     1000, &uav_odom_callback);
  cable_q_sub       = n.subscribe("cable_q_topic",      1000, &cable_q_callback);
  cable_q_dot_sub   = n.subscribe("cable_q_dot_topic",  1000, &cable_q_dot_callback);
  cable_w_sub       = n.subscribe("cable_w_topic",      1000, &cable_w_callback);
  payload_vel_sub   = n.subscribe("payload_vel_topic",  1000, &payload_vel_callback);
  u_i_sub           = n.subscribe("u_i_topic",          1000, &u_i_callback);
  u_i_parallel_sub  = n.subscribe("u_i_parallel_topic", 1000, &u_i_parallel_callback);
  u_i_perpendicular_sub = n.subscribe("u_i_perpendicular_topic",  1000, &u_i_perpendicular_callback);

  // Marker publishers
  payload_odom_pub  = n.advertise<visualization_msgs::Marker>("payload_odom_viz", 1000);
  cable_q_pub       = n.advertise<visualization_msgs::Marker>("cable_q_viz", 1000);
  cable_q_dot_pub   = n.advertise<visualization_msgs::Marker>("cable_q_dot_viz", 1000);
  cable_w_pub       = n.advertise<visualization_msgs::Marker>("cable_w_viz", 1000);
  payload_vel_pub   = n.advertise<visualization_msgs::Marker>("payload_vel_viz", 1000);
  u_i_pub           = n.advertise<visualization_msgs::Marker>("u_i_viz", 1000);
  u_i_parallel_pub  = n.advertise<visualization_msgs::Marker>("u_i_parallel_viz", 1000);
  u_i_perpendicular_pub = n.advertise<visualization_msgs::Marker>("u_i_perpendicular_viz", 1000);

  if(!n.getParam("cable_length", cable_length))
    ROS_WARN("couldn't load cable_length");
  if(!n.getParam("fixed_frame_id", fixed_frame_id))
    ROS_WARN("couldn't load ffi");

  ROS_INFO("cctv cl %2.2f", cable_length);
  ROS_INFO("cctv cl %s", fixed_frame_id.c_str());

  ros::spin();
  return 0;
}

void uav_odom_callback(nav_msgs::Odometry msg){
  uav_position(0) = msg.pose.pose.position.x;
  uav_position(1) = msg.pose.pose.position.y;
  uav_position(2) = msg.pose.pose.position.z;

  uav_position_point.x = msg.pose.pose.position.x;
  uav_position_point.y = msg.pose.pose.position.y;
  uav_position_point.z = msg.pose.pose.position.z;
}

void payload_odom_callback(nav_msgs::Odometry msg){
  // marker basics
  visualization_msgs::Marker marker;
  marker.ns = "payload_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // marker information from odom message
  marker.header = msg.header;
  marker.pose = msg.pose.pose;

  // publish marker
  payload_odom_pub.publish(marker);

  // store payload posiiton
  payload_position(0) = msg.pose.pose.position.x;
  payload_position(1) = msg.pose.pose.position.y;
  payload_position(2) = msg.pose.pose.position.z;

  payload_position_point.x = msg.pose.pose.position.x;
  payload_position_point.y = msg.pose.pose.position.y;
  payload_position_point.z = msg.pose.pose.position.z;

}
void cable_q_callback(geometry_msgs::Vector3Stamped msg){
  // marker basics
  visualization_msgs::Marker marker;
  marker.ns = "cable_q_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth
  marker.scale.y = 0.1;   // head girth
  marker.scale.z = 0.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // arrow base
  marker.points.push_back(uav_position_point);
  geometry_msgs::Point tip;
  tip.x = uav_position(0) + (msg.vector.x * cable_length);
  tip.y = uav_position(1) + (msg.vector.y * cable_length);
  tip.z = uav_position(2) + (msg.vector.z * cable_length);
  marker.points.push_back(tip);

  // marker information from odom message
  marker.header.frame_id = fixed_frame_id;
  marker.header.stamp = msg.header.stamp;
  marker.header.seq = msg.header.seq;

  // publish marker
  cable_q_pub.publish(marker);
}

void cable_q_dot_callback(geometry_msgs::Vector3Stamped msg){
  // marker basics
  visualization_msgs::Marker marker;
  marker.ns = "cable_q_dot_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth
  marker.scale.y = 0.1;   // head girth
  marker.scale.z = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // arrow base
  marker.points.push_back(uav_position_point);
  geometry_msgs::Point tip;
  tip.x = uav_position(0) + msg.vector.x;
  tip.y = uav_position(1) + msg.vector.y;
  tip.z = uav_position(2) + msg.vector.z;
  marker.points.push_back(tip);

  // marker information from odom message
  marker.header.frame_id = fixed_frame_id;
  marker.header.stamp = msg.header.stamp;
  marker.header.seq = msg.header.seq;

  // publish marker
  cable_q_dot_pub.publish(marker);
}

void cable_w_callback(geometry_msgs::Vector3Stamped msg){
  // marker basics
  visualization_msgs::Marker marker;
  marker.ns = "cable_w_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth
  marker.scale.y = 0.1;   // head girth
  marker.scale.z = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // arrow base
  marker.points.push_back(uav_position_point);
  geometry_msgs::Point tip;
  tip.x = uav_position(0) + msg.vector.x;
  tip.y = uav_position(1) + msg.vector.y;
  tip.z = uav_position(2) + msg.vector.z;
  marker.points.push_back(tip);

  // marker information from odom message
  marker.header.frame_id = fixed_frame_id;
  marker.header.stamp = msg.header.stamp;
  marker.header.seq = msg.header.seq;

  // publish marker
  cable_w_pub.publish(marker);
}

void payload_vel_callback(geometry_msgs::Vector3Stamped msg){
  // marker basics
  visualization_msgs::Marker marker;
  marker.ns = "payload_vel_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth
  marker.scale.y = 0.1;   // head girth
  marker.scale.z = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // arrow base
  marker.points.push_back(payload_position_point);
  geometry_msgs::Point tip;
  tip.x = payload_position(0) + msg.vector.x;
  tip.y = payload_position(1) + msg.vector.y;
  tip.z = payload_position(2) + msg.vector.z;
  marker.points.push_back(tip);

  // marker information from odom message
  marker.header.frame_id = fixed_frame_id;
  marker.header.stamp = msg.header.stamp;
  marker.header.seq = msg.header.seq;

  // publish marker
  payload_vel_pub.publish(marker);
}

void u_i_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // arrow base
  marker.points.push_back(uav_position_point);
  geometry_msgs::Point tip;
  tip.x = uav_position(0) + msg.vector.x;
  tip.y = uav_position(1) + msg.vector.y;
  tip.z = uav_position(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  u_i_pub.publish(marker);
}

void u_i_parallel_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // arrow base
  marker.points.push_back(uav_position_point);
  geometry_msgs::Point tip;
  tip.x = uav_position(0) + msg.vector.x;
  tip.y = uav_position(1) + msg.vector.y;
  tip.z = uav_position(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  u_i_parallel_pub.publish(marker);
}

void u_i_perpendicular_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // arrow base
  marker.points.push_back(uav_position_point);
  geometry_msgs::Point tip;
  tip.x = uav_position(0) + msg.vector.x;
  tip.y = uav_position(1) + msg.vector.y;
  tip.z = uav_position(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  u_i_perpendicular_pub.publish(marker);
}

visualization_msgs::Marker arrow_marker(std_msgs::Header msg_header){
  visualization_msgs::Marker marker;
  marker.ns = "basic_arrow";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth
  marker.scale.y = 0.1;   // head girth
  marker.scale.z = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // marker information from odom message
  marker.header.frame_id = fixed_frame_id;
  marker.header.stamp = msg_header.stamp;
  marker.header.seq = msg_header.seq;
}
