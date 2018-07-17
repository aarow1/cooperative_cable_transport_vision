#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <msgs_cctv/PayloadCommand.h>
#include <msgs_cctv/PayloadTrajCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <control_cctv/ControlCCTV.h>
#include <control_cctv/SO3Control.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

class NodeletControlCCTV : public nodelet::Nodelet
{
 public:
  NodeletControlCCTV() :
      position_cmd_updated_(false),
      position_cmd_init_(false),
      des_yaw_i_(0),
      des_yaw_dot_i_(0),
      current_yaw_(0),
      enable_motors_(false),
      use_external_yaw_(false),
      have_quad_odom_(false),
      have_payload_odom_(false),
      g_(9.81),
      quad_orr_(Eigen::Quaterniond::Identity())
  {
//    controller_.resetIntegrals();
  }

  void onInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishSO3Command(); // Still publish so3commands to attitude controller
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void payload_cmd_callback (const geometry_msgs::Pose::ConstPtr &payload_cmd);
  void payload_odom_callback(const nav_msgs::Odometry::ConstPtr &pl_odom);
  void quad_odom_callback(const nav_msgs::Odometry::ConstPtr &quad_odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);
  void use_cctv_control_callback(const std_msgs::Bool::ConstPtr &msg);

  void estimate_cable_state(void);
  void viz_cctv_control(void);

  // Keep two controllers to switch between them
  ControlCCTV cctv_controller_;
  SO3Control so3_controller_;

  ros::Publisher so3_command_pub_;
  ros::Subscriber payload_odom_sub_;
  ros::Subscriber quad_odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_;
  ros::Subscriber use_cctv_control_sub_;

  ros::Publisher rviz_marker_pub;
  ros::Publisher cable_q_pub, cable_q_dot_pub, cable_w_pub, payload_vel_pub;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  //Robot desired state (so3 controller)
  Eigen::Vector3d des_pos_, des_vel_, des_acc_, des_jrk_;
  double des_yaw_, des_yaw_dot_;

  ros::Subscriber payload_cmd_sub_;

  //SO3 controller gains
  Eigen::Vector3d kx_, kv_, ki_, kib_;

  //Payload desired state (cctv controller)
  Eigen::Vector3d des_pos_0_, des_vel_0_, des_acc_0_;
  Eigen::Matrix3d des_R_0_;
  Eigen::Vector3d des_Omega_0_, des_alpha_0_;

  double k_pos_0_, k_vel_0_, k_R_0_, k_Omega_0_, k_q_, k_w_;
  double des_yaw_i_, des_yaw_dot_i_;
  double current_yaw_;
  bool enable_motors_, use_external_yaw_, have_quad_odom_, have_payload_odom_;
  double kR_[3], kOm_[3], corrections_[3];

  //System definition
  const double g_;
  double robot_mass_;
  double payload_mass_;
  Eigen::Matrix3d J_0_;
  Eigen::Matrix3d J_i_;
  double cable_length_;
  Eigen::Matrix3d rho_;
  Eigen::Vector3d rho_i_; // my cable attachment point
  int idx_;
  int n_bots_;

  // Quad state
  Eigen::Vector3d     quad_pos_;
  Eigen::Vector3d     quad_vel_;
  Eigen::Quaterniond  quad_orr_;
  Eigen::Vector3d     quad_omg_;

  // Cable state
  Eigen::Vector3d     cable_q_;     // unit vector from robot to attachment point
  Eigen::Vector3d     cable_q_dot_;
  Eigen::Vector3d     cable_w_;

  // Payload state
  Eigen::Vector3d     pl_pos_;
  Eigen::Vector3d     pl_vel_;
  Eigen::Vector3d     pl_attach_;
  Eigen::Quaterniond  pl_orr_;
  Eigen::Vector3d     pl_omg_;

  // Filter constants
  double pl_vel_lowpass_alpha;
  double pl_vel_jumpmax;

  enum Controller {SO3_CONTROL, CCTV_CONTROL};
  Controller active_controller_ = SO3_CONTROL;
};


void NodeletControlCCTV::publishSO3Command()
{
  if (!have_quad_odom_)
  {
    ROS_WARN("No odometry! Not publishing SO3Command.");
    return;
  }

  Eigen::Vector3d ki = Eigen::Vector3d::Zero();
  Eigen::Vector3d kib = Eigen::Vector3d::Zero();
  if(enable_motors_)
  {
    ki = ki_;
    kib = kib_;
  }

  bool DB_CTRL_SWITCH = 1;

  quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command);
  so3_command->header.stamp = ros::Time::now();
  so3_command->header.frame_id = frame_id_;

  switch (active_controller_) {

  case SO3_CONTROL: {
    ROS_INFO_THROTTLE(1, "NodeletControlCCTV - Using so3 controller to publish s03 command");
    so3_controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, kib);

    const Eigen::Vector3d  &force         = so3_controller_.getComputedForce();
    const Eigen::Quaterniond &orientation = so3_controller_.getComputedOrientation();
    const Eigen::Vector3d &ang_vel        = so3_controller_.getComputedAngularVelocity();

    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = orientation.x();
    so3_command->orientation.y = orientation.y();
    so3_command->orientation.z = orientation.z();
    so3_command->orientation.w = orientation.w();
    so3_command->angular_velocity.x = ang_vel(0);
    so3_command->angular_velocity.y = ang_vel(1);
    so3_command->angular_velocity.z = ang_vel(2);

    cctv_controller_.calculateControl(des_pos_0_,
                               des_vel_0_,
                               des_acc_0_,
                               des_R_0_,
                               des_Omega_0_,
                               des_alpha_0_,
                               des_yaw_i_,
                               k_pos_0_,
                               k_vel_0_,
                               k_R_0_,
                               k_Omega_0_,
                               k_q_,
                               k_w_);

//    viz_cctv_control();

    break;
  }
  case CCTV_CONTROL: {
    ROS_DEBUG_THROTTLE(1, "NodeletControlCCTV - Using cctv controller to publish s03 command");
    cctv_controller_.calculateControl(des_pos_0_,
                               des_vel_0_,
                               des_acc_0_,
                               des_R_0_,
                               des_Omega_0_,
                               des_alpha_0_,
                               des_yaw_i_,
                               k_pos_0_,
                               k_vel_0_,
                               k_R_0_,
                               k_Omega_0_,
                               k_q_,
                               k_w_);

    const Eigen::Vector3d &force          = cctv_controller_.getComputedForce();
    const Eigen::Quaterniond &orientation = cctv_controller_.getComputedOrientation();
    const Eigen::Vector3d &ang_vel        = cctv_controller_.getComputedAngularVelocity();

    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = orientation.x();
    so3_command->orientation.y = orientation.y();
    so3_command->orientation.z = orientation.z();
    so3_command->orientation.w = orientation.w();
    so3_command->angular_velocity.x = ang_vel(0);
    so3_command->angular_velocity.y = ang_vel(1);
    so3_command->angular_velocity.z = ang_vel(2);
    break;
  }

  default:
    ROS_ERROR("Control nodelet isn't sure which controller to use");
    break;
  }


  for(int i = 0; i < 3; i++)
  {
    so3_command->kR[i] = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  so3_command->aux.current_yaw = current_yaw_;
  so3_command->aux.kf_correction = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors = enable_motors_;
  so3_command->aux.use_external_yaw = use_external_yaw_;
  so3_command_pub_.publish(so3_command);
}

void NodeletControlCCTV::viz_cctv_control(){
  visualization_msgs::MarkerArray marker_array_msg;

  geometry_msgs::Point base;
  base.x = quad_pos_(0);
  base.y = quad_pos_(1);
  base.z = quad_pos_(2);

  // Net desired force arrow (green)
  visualization_msgs::Marker force_marker_msg;
  force_marker_msg.header.stamp = ros::Time::now();
  force_marker_msg.header.frame_id = "simulator";
  force_marker_msg.ns = "force_arrow";
  force_marker_msg.type = visualization_msgs::Marker::ARROW;
  force_marker_msg.action = visualization_msgs::Marker::ADD;
  force_marker_msg.lifetime = ros::Duration();
  force_marker_msg.scale.x = 0.05;   // shaft diameter
  force_marker_msg.scale.y = 0.1;   // head diameter
  force_marker_msg.color.r = 0.0;
  force_marker_msg.color.g = 1.0;
  force_marker_msg.color.b = 0.0;
  force_marker_msg.color.a = 1.0;

  const double vec_scale = 0.05;

  geometry_msgs::Point force_tip;
  const Eigen::Vector3d &cctv_force = cctv_controller_.getComputedForce();
  force_tip.x = quad_pos_(0) + (cctv_force(0)*vec_scale);
  force_tip.y = quad_pos_(1) + (cctv_force(1)*vec_scale);
  force_tip.z = quad_pos_(2) + (cctv_force(2)*vec_scale);
  force_marker_msg.points.push_back(base);
  force_marker_msg.points.push_back(force_tip);

  // Parallel component of force (yellow)
  visualization_msgs::Marker parallel_marker;
  parallel_marker.header.stamp = ros::Time::now();
  parallel_marker.header.frame_id = "simulator";
  parallel_marker.ns = "parallel_marker";
  parallel_marker.type = visualization_msgs::Marker::ARROW;
  parallel_marker.action = visualization_msgs::Marker::ADD;
  parallel_marker.lifetime = ros::Duration();
  parallel_marker.scale.x = 0.05;   // shaft diameter
  parallel_marker.scale.y = 0.1;   // head diameter
  parallel_marker.color.r = 1.0;
  parallel_marker.color.g = 1.0;
  parallel_marker.color.b = 0.0;
  parallel_marker.color.a = 1.0;
  geometry_msgs::Point parallel_tip;
  const Eigen::Vector3d &cctv_parallel = cctv_controller_.get_u_i_parallel();
  parallel_tip.x = quad_pos_(0) + (cctv_parallel(0)*vec_scale);
  parallel_tip.y = quad_pos_(1) + (cctv_parallel(1)*vec_scale);
  parallel_tip.z = quad_pos_(2) + (cctv_parallel(2)*vec_scale);
  parallel_marker.points.push_back(base);
  parallel_marker.points.push_back(parallel_tip);

  // Perpendicular component of force (blue)
  visualization_msgs::Marker perpendicular_marker;
  perpendicular_marker.header.stamp = ros::Time::now();
  perpendicular_marker.header.frame_id = "simulator";
  perpendicular_marker.ns = "perpendicular_marker";
  perpendicular_marker.type = visualization_msgs::Marker::ARROW;
  perpendicular_marker.action = visualization_msgs::Marker::ADD;
  perpendicular_marker.lifetime = ros::Duration();
  perpendicular_marker.scale.x = 0.05;   // shaft diameter
  perpendicular_marker.scale.y = 0.1;   // head diameter
  perpendicular_marker.color.r = 0.0;
  perpendicular_marker.color.g = 0.0;
  perpendicular_marker.color.b = 1.0;
  perpendicular_marker.color.a = 1.0;
  geometry_msgs::Point perpendicular_tip;
  const Eigen::Vector3d &cctv_perpendicular = cctv_controller_.get_u_i_perpendicular();
  perpendicular_tip.x = quad_pos_(0) + (cctv_perpendicular(0)*vec_scale);
  perpendicular_tip.y = quad_pos_(1) + (cctv_perpendicular(1)*vec_scale);
  perpendicular_tip.z = quad_pos_(2) + (cctv_perpendicular(2)*vec_scale);
  perpendicular_marker.points.push_back(base);
  perpendicular_marker.points.push_back(perpendicular_tip);

  // attachment point
  visualization_msgs::Marker attach_marker;
  attach_marker.header.stamp = ros::Time::now();
  attach_marker.header.frame_id = "simulator";
  attach_marker.ns = "attachment_point";
  attach_marker.type = visualization_msgs::Marker::SPHERE;
  attach_marker.action = visualization_msgs::Marker::ADD;
  attach_marker.lifetime = ros::Duration();
  attach_marker.pose.position.x = pl_attach_(0);
  attach_marker.pose.position.y = pl_attach_(1);
  attach_marker.pose.position.z = pl_attach_(2);
  attach_marker.scale.x = 0.1;
  attach_marker.scale.y = 0.1;
  attach_marker.scale.z = 0.1;
  attach_marker.color.r = 0.0;
  attach_marker.color.g = 0.0;
  attach_marker.color.b = 1.0;
  attach_marker.color.a = 1.0;

  // Add all markers to marker array
  marker_array_msg.markers.push_back(force_marker_msg);
  marker_array_msg.markers.push_back(parallel_marker);
  marker_array_msg.markers.push_back(perpendicular_marker);
  marker_array_msg.markers.push_back(attach_marker);
  rviz_marker_pub.publish(marker_array_msg);
}

void NodeletControlCCTV::payload_odom_callback(const nav_msgs::Odometry::ConstPtr &pl_odom)
{
  pl_pos_(0)  = pl_odom->pose.pose.position.x;
  pl_pos_(1)  = pl_odom->pose.pose.position.y;
  pl_pos_(2)  = pl_odom->pose.pose.position.z;
  pl_orr_.x() = pl_odom->pose.pose.orientation.x;
  pl_orr_.y() = pl_odom->pose.pose.orientation.y;
  pl_orr_.z() = pl_odom->pose.pose.orientation.z;
  pl_orr_.w() = pl_odom->pose.pose.orientation.w;
  pl_omg_(0)  = pl_odom->twist.twist.angular.x;
  pl_omg_(1)  = pl_odom->twist.twist.angular.y;
  pl_omg_(2)  = pl_odom->twist.twist.angular.z;

  Eigen::Vector3d pl_vel_in;
  pl_vel_in(0)  = pl_odom->twist.twist.linear.x;
  pl_vel_in(1)  = pl_odom->twist.twist.linear.y;
  pl_vel_in(2)  = pl_odom->twist.twist.linear.z;

  const Eigen::Vector3d pl_vel_diff = pl_vel_in - pl_vel_;

  static int consecutive_jumps = 0;

  if (pl_vel_diff.norm() < pl_vel_jumpmax)
    pl_vel_ = pl_vel_ + pl_vel_lowpass_alpha * (pl_vel_in - pl_vel_);
  else {
    consecutive_jumps++;
    if(consecutive_jumps >= 5){
      pl_vel_ = pl_vel_ + pl_vel_lowpass_alpha * (pl_vel_in - pl_vel_);
    }
  }

  // Set payload state using controller accessors
  cctv_controller_.set_pos_0  (pl_pos_);
  cctv_controller_.set_vel_0  (pl_vel_);
  cctv_controller_.set_R_0    (pl_orr_.normalized().toRotationMatrix());
  cctv_controller_.set_Omega_0(pl_omg_);
  estimate_cable_state();

//  publishSO3Command();
}


void NodeletControlCCTV::quad_odom_callback(const nav_msgs::Odometry::ConstPtr &quad_odom)
{
  have_quad_odom_ = true;

  quad_pos_(0) = quad_odom->pose.pose.position.x;
  quad_pos_(1) = quad_odom->pose.pose.position.y;
  quad_pos_(2) = quad_odom->pose.pose.position.z;
  quad_vel_(0) = quad_odom->twist.twist.linear.x;
  quad_vel_(1) = quad_odom->twist.twist.linear.y;
  quad_vel_(2) = quad_odom->twist.twist.linear.z;
  quad_orr_.x() = quad_odom->pose.pose.orientation.x;
  quad_orr_.y() = quad_odom->pose.pose.orientation.y;
  quad_orr_.z() = quad_odom->pose.pose.orientation.z;
  quad_orr_.w() = quad_odom->pose.pose.orientation.w;
  quad_omg_(0) = quad_odom->twist.twist.angular.x;
  quad_omg_(1) = quad_odom->twist.twist.angular.y;
  quad_omg_(2) = quad_odom->twist.twist.angular.z;

  current_yaw_ = tf::getYaw(quad_odom->pose.pose.orientation);

  so3_controller_.setPosition(quad_pos_);
  so3_controller_.setVelocity(quad_vel_);
  so3_controller_.setCurrentOrientation(quad_orr_);

  cctv_controller_.set_R_i(quad_orr_.normalized().toRotationMatrix());
  cctv_controller_.set_Omega_i(quad_omg_);
  estimate_cable_state();


  if(position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some time
    if(!position_cmd_updated_){
      publishSO3Command();
//      ROS_ERROR_THROTTLE(0.25, "CONFUSING FLAG FUNCTION PUBLISHED SO3");
    }
    position_cmd_updated_ = false;
  }
//  publishSO3Command();
}

//------------------------------------------------------------------------------------------------
// Hacky stuff about cable angle, todo NEEDS TO BE DONE BETTER
//------------------------------------------------------------------------------------------------
void NodeletControlCCTV::estimate_cable_state(void){

  // Note: p = location, v = velocity, p_b_0 = position, of(sub) b, in-frame(super) 0

  // 1. calculate q_i
  const Eigen::Matrix3d R_pl_0 = pl_orr_.normalized().toRotationMatrix();
  const Eigen::Vector3d p_attach_0 = pl_pos_ + (R_pl_0 * rho_i_);
  pl_attach_ = p_attach_0;
  const Eigen::Vector3d p_attach_quad = (p_attach_0 - quad_pos_);
  if(p_attach_quad.norm() > 0.01){
    cable_q_ = p_attach_quad.normalized();
  }
//  cable_q_.normalize();

  // 2. calculate q_i_dot

  const Eigen::Vector3d v_attach_0 = pl_vel_ + pl_omg_.cross(rho_i_);
  const Eigen::Vector3d v_attach_quad = v_attach_0 - quad_vel_;
  cable_q_dot_ = v_attach_quad / cable_length_;

//  ROS_WARN_THROTTLE(1, "nodelet pl_omg_: [%2.2f, %2.2f, %2.2f]",
//                    pl_omg_(0),
//                    pl_omg_(1),
//                    pl_omg_(2));
//  ROS_WARN_THROTTLE(.01, "v_attach_0: [%2.2f, %2.2f, %2.2f]",
//                    v_attach_0(0),
//                    v_attach_0(1),
//                    v_attach_0(2));
//  ROS_WARN_THROTTLE(1, "nodelet quad_vel_: [%2.2f, %2.2f, %2.2f]",
//                    quad_vel_(0),
//                    quad_vel_(1),
//                    quad_vel_(2));
//  ROS_WARN_THROTTLE(.01, "v_attach_quad: [%2.2f, %2.2f, %2.2f]",
//                    v_attach_quad(0),
//                    v_attach_quad(1),
//                    v_attach_quad(2));

//  ROS_WARN_THROTTLE(.01, "cable_q_dot: [%2.2f, %2.2f, %2.2f]",
//                    v_attach_quad(0),
//                    v_attach_quad(1),
//                    v_attach_quad(2));

  // 3. calculate w_i (yikes)
  cable_w_ = (p_attach_quad.cross(v_attach_quad)) / (cable_length_ * cable_length_);

  cctv_controller_.set_q_i(cable_q_);
  cctv_controller_.set_q_i_dot(cable_q_dot_);
  cctv_controller_.set_w_i(cable_w_);

  // publish cable state for debug
  geometry_msgs::Vector3Stamped cable_q_msg;
  cable_q_msg.header.stamp = ros::Time::now();
  cable_q_msg.header.frame_id = frame_id_;
  cable_q_msg.vector.x = cable_q_(0);
  cable_q_msg.vector.y = cable_q_(1);
  cable_q_msg.vector.z = cable_q_(2);
  cable_q_pub.publish(cable_q_msg);

  geometry_msgs::Vector3Stamped cable_q_dot_msg;
  cable_q_dot_msg.header.stamp = ros::Time::now();
  cable_q_dot_msg.header.frame_id = frame_id_;
  cable_q_dot_msg.vector.x = cable_q_dot_(0);
  cable_q_dot_msg.vector.y = cable_q_dot_(1);
  cable_q_dot_msg.vector.z = cable_q_dot_(2);
  cable_q_dot_pub.publish(cable_q_dot_msg);

  geometry_msgs::Vector3Stamped cable_w_msg;
  cable_w_msg.header.stamp = ros::Time::now();
  cable_w_msg.header.frame_id = frame_id_;
  cable_w_msg.vector.x = cable_w_(0);
  cable_w_msg.vector.y = cable_w_(1);
  cable_w_msg.vector.z = cable_w_(2);
  cable_w_pub.publish(cable_w_msg);

  geometry_msgs::Vector3Stamped payload_vel_msg;
  payload_vel_msg.header.stamp = ros::Time::now();
  payload_vel_msg.header.frame_id = frame_id_;
  payload_vel_msg.vector.x = pl_vel_(0);
  payload_vel_msg.vector.y = pl_vel_(1);
  payload_vel_msg.vector.z = pl_vel_(2);
  payload_vel_pub.publish(payload_vel_msg);

}

void NodeletControlCCTV::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  des_jrk_ = Eigen::Vector3d(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_ = true;

  publishSO3Command();
}

void NodeletControlCCTV::payload_cmd_callback(const geometry_msgs::Pose::ConstPtr &payload_cmd)
{
  des_pos_0_ = Eigen::Vector3d(payload_cmd->position.x, payload_cmd->position.x, payload_cmd->position.x);
  Eigen::Quaterniond cmd_quaternion = Eigen::Quaterniond(payload_cmd->orientation.x,
                                      payload_cmd->orientation.y,
                                      payload_cmd->orientation.z,
                                      payload_cmd->orientation.w);
  des_R_0_ = cmd_quaternion.normalized().toRotationMatrix();
  ROS_INFO("~~~got new payload command position~~~~");
}


void NodeletControlCCTV::enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Control cctv Enabling motors");
  else
    ROS_INFO("control cctv Disabling motors");

  enable_motors_ = msg->data;
  // Reset integral when toggling motor state
//  controller_.resetIntegrals();
}

void NodeletControlCCTV::use_cctv_control_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data){
    ROS_INFO("Switching to cctv control... good luck");
    active_controller_ = CCTV_CONTROL;
  }
  else{
    ROS_INFO("Switching back to so3 control");
    active_controller_ = SO3_CONTROL;
  }

  // Reset integral when toggling motor state
//  controller_.resetIntegrals();
}

void NodeletControlCCTV::corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void NodeletControlCCTV::onInit()
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  std::string quadrotor_name;
  priv_nh.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  // Retrieve robot params
  priv_nh.param("robot_mass", robot_mass_, 0.5);
  priv_nh.param("use_external_yaw", use_external_yaw_, true);
  priv_nh.param("gains/ki/x", ki_[0], 0.0);
  priv_nh.param("gains/ki/y", ki_[1], 0.0);
  priv_nh.param("gains/ki/z", ki_[2], 0.0);
  priv_nh.param("gains/kib/x", kib_[0], 0.0);
  priv_nh.param("gains/kib/y", kib_[1], 0.0);
  priv_nh.param("gains/kib/z", kib_[2], 0.0);
  priv_nh.param("gains/rot/x", kR_[0], 1.5);
  priv_nh.param("gains/rot/y", kR_[1], 1.5);
  priv_nh.param("gains/rot/z", kR_[2], 1.0);
  priv_nh.param("gains/ang/x", kOm_[0], 0.13);
  priv_nh.param("gains/ang/y", kOm_[1], 0.13);
  priv_nh.param("gains/ang/z", kOm_[2], 0.1);
  priv_nh.param("corrections/kf", corrections_[0], 0.0);
  priv_nh.param("corrections/r", corrections_[1], 0.0);
  priv_nh.param("corrections/p", corrections_[2], 0.0);
  double max_pos_int, max_pos_int_b;
  priv_nh.param("max_pos_int", max_pos_int, 0.5);
  priv_nh.param("mas_pos_int_b", max_pos_int_b, 0.5);
  float max_tilt_angle;
  priv_nh.param("max_tilt_angle", max_tilt_angle, static_cast<float>(M_PI));

  // Set so3 controller params
  so3_controller_.setMass(robot_mass_);
  so3_controller_.setGravity(g_);
  so3_controller_.setMaxIntegral(max_pos_int);
  so3_controller_.setMaxIntegralBody(max_pos_int_b);
  so3_controller_.setMaxTiltAngle(max_tilt_angle);

  // Get payload params
  priv_nh.param("payload_mass", payload_mass_, 1.0);
  priv_nh.param("cable_length", cable_length_, 0.33);
  priv_nh.param("number_of_robots", n_bots_, 5);
  priv_nh.param("my_index", idx_, 0);
  // Get Payload gains
  priv_nh.param("k_pos_0", k_pos_0_,      0.0);
  priv_nh.param("k_vel_0", k_vel_0_,      0.0);
  priv_nh.param("k_R_0", k_R_0_,          0.0);
  priv_nh.param("k_Omega_0", k_Omega_0_,  0.0);
  priv_nh.param("k_q", k_q_,              0.0);
  priv_nh.param("k_w", k_w_,              0.0);

  priv_nh.param("pl_vel_lowpass_alpha", pl_vel_lowpass_alpha,  0.05);
  priv_nh.param("pl_vel_jumpmax", pl_vel_jumpmax,  0.1);

  // Retrieve attachment point params for all robots
  for (int i=0; i<n_bots_; i++){
    ROS_INFO("loading rho %i", i);
    std::string param_name = "attachment/rho_" + std::to_string(i);
    if(!priv_nh.getParam(param_name + "/x", rho_(0, i)))
      ROS_ERROR("Couldn't find param: %s/x", param_name.c_str());
    if(!priv_nh.getParam(param_name + "/y", rho_(1, i)))
      ROS_ERROR("Couldn't find param: %s/y", param_name.c_str());
    if(!priv_nh.getParam(param_name + "/z", rho_(2, i)))
      ROS_ERROR("Couldn't find param: %s/z", param_name.c_str());
  }
  rho_i_ = rho_.block<3,1>(0, idx_);

  // Set cctv controller params
  cctv_controller_.set_m_0(payload_mass_);
  cctv_controller_.set_J_0(Eigen::Matrix3d::Identity());
  cctv_controller_.set_m_i(robot_mass_);
  cctv_controller_.set_J_i(Eigen::Matrix3d::Identity());
  cctv_controller_.set_l_i(cable_length_);
  cctv_controller_.set_rho(rho_);
  cctv_controller_.set_g  (g_);
  cctv_controller_.set_idx(idx_);
  cctv_controller_.set_n_bots(n_bots_);

  // Set cctv_controller des to zero TODO not this

  des_pos_0_.setZero();
  des_pos_0_(2) = 0.1;
  des_vel_0_.setZero();
  des_acc_0_.setZero();
  des_R_0_.setIdentity();
  des_Omega_0_.setZero();
  des_alpha_0_.setZero();

  so3_command_pub_    = priv_nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  rviz_marker_pub     = priv_nh.advertise<visualization_msgs::MarkerArray>("cctv_controller_marker_array", 10);
  cable_q_pub         = priv_nh.advertise<geometry_msgs::Vector3Stamped>("cable_q", 10);
  cable_q_dot_pub     = priv_nh.advertise<geometry_msgs::Vector3Stamped>("cable_q_dot", 10);
  cable_w_pub         = priv_nh.advertise<geometry_msgs::Vector3Stamped>("cable_w", 10);
  payload_vel_pub     = priv_nh.advertise<geometry_msgs::Vector3Stamped>("payload_vel", 10);

  payload_odom_sub_   = priv_nh.subscribe("payload_odom", 10, &NodeletControlCCTV::payload_odom_callback,         this, ros::TransportHints().tcpNoDelay());
  quad_odom_sub_      = priv_nh.subscribe("quad_odom",    10, &NodeletControlCCTV::quad_odom_callback,            this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_   = priv_nh.subscribe("position_cmd", 10, &NodeletControlCCTV::position_cmd_callback,         this, ros::TransportHints().tcpNoDelay());
  enable_motors_sub_  = priv_nh.subscribe("motors",       2,  &NodeletControlCCTV::enable_motors_callback,        this, ros::TransportHints().tcpNoDelay());
  corrections_sub_    = priv_nh.subscribe("corrections",  10, &NodeletControlCCTV::corrections_callback,          this, ros::TransportHints().tcpNoDelay());
  payload_cmd_sub_    = priv_nh.subscribe("payload_cmd",  10, &NodeletControlCCTV::payload_cmd_callback,          this, ros::TransportHints().tcpNoDelay());

  use_cctv_control_sub_ = priv_nh.subscribe("use_cctv_controller", 10, &NodeletControlCCTV::use_cctv_control_callback, this);

  ROS_INFO("nodelet control cctv is alive");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NodeletControlCCTV, nodelet::Nodelet);
