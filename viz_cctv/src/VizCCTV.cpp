#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Geometry>
#include <msgs_cctv/PayloadTrajCommand.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

// Global variables
ros::Subscriber payload_odom_sub;
ros::Subscriber quad_odom_sub;

ros::Subscriber q_i_sub;
ros::Subscriber q_i_dot_sub;
ros::Subscriber w_i_sub;
ros::Subscriber u_i_sub;
ros::Subscriber u_i_prl_sub;
ros::Subscriber u_i_prp_sub;

ros::Subscriber F_0_des_sub;
ros::Subscriber M_0_des_sub;
ros::Subscriber q_i_des_sub;

ros::Publisher  payload_odom_pub;
ros::Publisher  quad_odom_pub;
ros::Publisher  q_i_pub;
ros::Publisher  q_i_dot_pub;
ros::Publisher  w_i_pub;
ros::Publisher  u_i_pub;
ros::Publisher  u_i_prl_pub;
ros::Publisher  u_i_prp_pub;

ros::Publisher F_0_des_pub;
ros::Publisher M_0_des_pub;
ros::Publisher q_i_des_pub;

Eigen::Vector3f quad_pos, payload_pos;
geometry_msgs::Point quad_pos_point, payload_pos_point;
double cable_length;
std::string fixed_frame_id, quad_name;

// Function prototypes
void payload_odom_callback( nav_msgs::Odometry msg);
void quad_odom_callback(    nav_msgs::Odometry msg);
void q_i_callback(          geometry_msgs::Vector3Stamped msg);
void q_i_dot_callback(      geometry_msgs::Vector3Stamped msg);
void w_i_callback(          geometry_msgs::Vector3Stamped msg);
void u_i_callback(          geometry_msgs::Vector3Stamped msg);
void u_i_prl_callback(      geometry_msgs::Vector3Stamped msg);
void u_i_prp_callback(      geometry_msgs::Vector3Stamped msg);
void F_0_des_callback(      geometry_msgs::Vector3Stamped msg);
void M_0_des_callback(      geometry_msgs::Vector3Stamped msg);
void q_i_des_callback(      geometry_msgs::Vector3Stamped msg);

visualization_msgs::Marker arrow_marker(std_msgs::Header msg_header);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viz_cctv");

  ros::NodeHandle n("~");

  n.getParam("quad_name", quad_name);
  // Subscribers
  payload_odom_sub  = n.subscribe("payload_odom_topic", 1000, &payload_odom_callback);
  quad_odom_sub     = n.subscribe("quad_odom_topic",     1000, &quad_odom_callback);

  q_i_sub           = n.subscribe("q_i_topic",      1000, &q_i_callback);
  q_i_dot_sub       = n.subscribe("q_i_dot_topic",  1000, &q_i_dot_callback);
  w_i_sub           = n.subscribe("w_i_topic",      1000, &w_i_callback);
  u_i_sub           = n.subscribe("u_i_topic",          1000, &u_i_callback);
  u_i_prl_sub       = n.subscribe("u_i_prl_topic", 1000, &u_i_prl_callback);
  u_i_prp_sub       = n.subscribe("u_i_prp_topic",  1000, &u_i_prp_callback);

  F_0_des_sub       = n.subscribe("F_0_des_topic",  1000, &F_0_des_callback);
  M_0_des_sub       = n.subscribe("M_0_des_topic",  1000, &M_0_des_callback);
  q_i_des_sub       = n.subscribe("q_i_des_topic",  1000, &q_i_des_callback);

  // Marker publishers
  payload_odom_pub  = n.advertise<visualization_msgs::Marker>("payload_odom_viz", 1000);
  q_i_pub           = n.advertise<visualization_msgs::Marker>("q_i_viz", 1000);
  q_i_dot_pub       = n.advertise<visualization_msgs::Marker>("q_i_dot_viz", 1000);
  w_i_pub           = n.advertise<visualization_msgs::Marker>("w_i_viz", 1000);
  u_i_pub           = n.advertise<visualization_msgs::Marker>("u_i_viz", 1000);
  u_i_prl_pub       = n.advertise<visualization_msgs::Marker>("u_i_prl_viz", 1000);
  u_i_prp_pub       = n.advertise<visualization_msgs::Marker>("u_i_prp_viz", 1000);

  F_0_des_pub       = n.advertise<visualization_msgs::Marker>("F_0_des_viz", 1000);
  M_0_des_pub       = n.advertise<visualization_msgs::Marker>("M_0_des_viz", 1000);
  q_i_des_pub       = n.advertise<visualization_msgs::Marker>("q_i_des_viz", 1000);

  if(!n.getParam("cable_length", cable_length))
    ROS_WARN("viz cctv couldn't load cable_length");
  if(!n.getParam("fixed_frame_id", fixed_frame_id))
    ROS_WARN("viz cctv couldn't load ffi");

  ROS_INFO("cctv cl %2.2f", cable_length);
  ROS_INFO("cctv cl %s", fixed_frame_id.c_str());

  ros::spin();
  return 0;
}

void quad_odom_callback(nav_msgs::Odometry msg){
  quad_pos(0) = msg.pose.pose.position.x;
  quad_pos(1) = msg.pose.pose.position.y;
  quad_pos(2) = msg.pose.pose.position.z;

  quad_pos_point.x = msg.pose.pose.position.x;
  quad_pos_point.y = msg.pose.pose.position.y;
  quad_pos_point.z = msg.pose.pose.position.z;
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
  payload_pos(0) = msg.pose.pose.position.x;
  payload_pos(1) = msg.pose.pose.position.y;
  payload_pos(2) = msg.pose.pose.position.z;

  payload_pos_point.x = msg.pose.pose.position.x;
  payload_pos_point.y = msg.pose.pose.position.y;
  payload_pos_point.z = msg.pose.pose.position.z;

}
void q_i_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1;
  marker.scale.x = 0.02;  // shaft girth, tiny

  // arrow base
  marker.points.push_back(quad_pos_point);
  geometry_msgs::Point tip;
  tip.x = quad_pos(0) + msg.vector.x;
  tip.y = quad_pos(1) + msg.vector.y;
  tip.z = quad_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  q_i_pub.publish(marker);
}

void q_i_des_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 0.5;
  marker.scale.x = 0.1;  // shaft girth, wide

  // SPECIAL arrow construction (base is at where the quad should be)
  geometry_msgs::Point base;
  base.x = payload_pos(0) - msg.vector.x;
  base.y = payload_pos(1) - msg.vector.y;
  base.z = payload_pos(2) - msg.vector.z;
  marker.points.push_back(base);
  marker.points.push_back(payload_pos_point);

  // publish marker
  q_i_des_pub.publish(marker);
}

void q_i_dot_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // arrow base
  marker.points.push_back(quad_pos_point);
  geometry_msgs::Point tip;
  tip.x = quad_pos(0) + msg.vector.x;
  tip.y = quad_pos(1) + msg.vector.y;
  tip.z = quad_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  q_i_dot_pub.publish(marker);
}

void w_i_callback(geometry_msgs::Vector3Stamped msg){
  // marker basics
  visualization_msgs::Marker marker;
  marker.ns = "w_i_marker";
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
  marker.points.push_back(quad_pos_point);
  geometry_msgs::Point tip;
  tip.x = quad_pos(0) + msg.vector.x;
  tip.y = quad_pos(1) + msg.vector.y;
  tip.z = quad_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // marker information from odom message
  marker.header.frame_id = fixed_frame_id;
  marker.header.stamp = msg.header.stamp;
  marker.header.seq = msg.header.seq;

  // publish marker
  w_i_pub.publish(marker);
}

void u_i_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth, wide
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // arrow base
  marker.points.push_back(quad_pos_point);
  geometry_msgs::Point tip;
  tip.x = quad_pos(0) + msg.vector.x;
  tip.y = quad_pos(1) + msg.vector.y;
  tip.z = quad_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  u_i_pub.publish(marker);
}

void u_i_prl_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth, small
  marker.color.r = 1.0;
  marker.color.g = 0.7;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  // arrow base
  marker.points.push_back(quad_pos_point);
  geometry_msgs::Point tip;
  tip.x = quad_pos(0) + msg.vector.x;
  tip.y = quad_pos(1) + msg.vector.y;
  tip.z = quad_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  u_i_prl_pub.publish(marker);
}

void u_i_prp_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // marker appearance
  marker.scale.x = 0.05;  // shaft girth, small
  marker.color.r = 0.7;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  // arrow base
  marker.points.push_back(quad_pos_point);
  geometry_msgs::Point tip;
  tip.x = quad_pos(0) + msg.vector.x;
  tip.y = quad_pos(1) + msg.vector.y;
  tip.z = quad_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  u_i_prp_pub.publish(marker);
}

void F_0_des_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // arrow base
  marker.points.push_back(payload_pos_point);
  geometry_msgs::Point tip;
  tip.x = payload_pos(0) + msg.vector.x;
  tip.y = payload_pos(1) + msg.vector.y;
  tip.z = payload_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  F_0_des_pub.publish(marker);
}

void M_0_des_callback(geometry_msgs::Vector3Stamped msg){
  // create basic marker
  visualization_msgs::Marker marker = arrow_marker(msg.header);

  // arrow base
  marker.points.push_back(payload_pos_point);
  geometry_msgs::Point tip;
  tip.x = payload_pos(0) + msg.vector.x;
  tip.y = payload_pos(1) + msg.vector.y;
  tip.z = payload_pos(2) + msg.vector.z;
  marker.points.push_back(tip);

  // publish marker
  M_0_des_pub.publish(marker);
}


visualization_msgs::Marker arrow_marker(std_msgs::Header msg_header){
  visualization_msgs::Marker marker;
  marker.ns = "basic_arrow";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(10);

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

  return marker;
}
