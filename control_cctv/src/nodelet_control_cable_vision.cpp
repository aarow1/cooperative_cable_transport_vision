#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <msgs_cctv/PayloadCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <control_cctv/ControlCCTV.h>
#include <tf/transform_datatypes.h>

class ControlCCTVNodelet : public nodelet::Nodelet
{
 public:
  ControlCCTVNodelet() :
      position_cmd_updated_(false),
      position_cmd_init_(false),
      des_yaw_i_(0),
      des_yaw_dot_i_(0),
      current_yaw_(0),
      enable_motors_(false),
      use_external_yaw_(false),
      have_odom_(false),
      g_(9.81),
      current_orientation_(Eigen::Quaternionf::Identity())
  {
//    controller_.resetIntegrals();
  }

  void onInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishSO3Command(); // Still publish so3commands to attitude controller
  void payload_position_cmd_callback(const msgs_cctv::PayloadCommandConstPtr &cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);

  ControlCableVision controller_;
  ros::Publisher so3_command_pub_;
  ros::Subscriber odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_0_, des_vel_0_, des_acc_0_;
  Eigen::Matrix3d des_R_0_;
  Eigen::Vector3d des_Omega_0_, des_alpha_0_;

  Eigen::Vector3d kx_, kv_, ki_, kib_;
  double k_pos_0_, k_vel_0_, k_R_0_, k_Omega_0_, k_q_, k_w_;
  double des_yaw_i_, des_yaw_dot_i_;
  float current_yaw_;
  bool enable_motors_, use_external_yaw_, have_odom_;
  float kR_[3], kOm_[3], corrections_[3];
  float mass_;
  const float g_;
  Eigen::Quaternionf current_orientation_;
};


void ControlCableVisionNodelet::publishSO3Command()
{
  if (!have_odom_)
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

  controller_.calculateControl(des_pos_0_,
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

  const Eigen::Vector3d &force = controller_.getComputedForce();
  const Eigen::Quaterniond &orientation = controller_.getComputedOrientation();
  const Eigen::Vector3d &ang_vel = controller_.getComputedAngularVelocity();

  quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command);
  so3_command->header.stamp = ros::Time::now();
  so3_command->header.frame_id = frame_id_;
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

void ControlCableVisionNodelet::payload_position_cmd_callback(const msgs_cctv::PayloadCommandConstPtr &cmd)
{
  des_pos_0_ = Eigen::Vector3d( cmd->payload_position.x,
                                cmd->payload_position.y,
                                cmd->payload_position.z);
  des_vel_0_ = Eigen::Vector3d( cmd->payload_velocity.x,
                                cmd->payload_velocity.y,
                                cmd->payload_velocity.z);
  des_acc_0_ = Eigen::Vector3d( cmd->payload_acceleration.x,
                                cmd->payload_acceleration.y,
                                cmd->payload_acceleration.z);
  des_R_0_   = Eigen::Quaterniond(cmd->payload_orientation.w,
                                  cmd->payload_orientation.x,
                                  cmd->payload_orientation.y,
                                  cmd->payload_orientation.z);
  des_Omega_0_ = Eigen::Vector3d( cmd->payload_angular_velocity.x,
                                cmd->payload_angular_velocity.y,
                                cmd->payload_angular_velocity.z);
  des_alpha_0_ = Eigen::Vector3d( cmd->payload_angular_acceleration.x,
                                cmd->payload_angular_acceleration.y,
                                cmd->payload_angular_acceleration.z);
  k_pos_0_  = cmd->k_x_payload;
  k_vel_0_  = cmd->k_v_payload;
  k_R_0_    = cmd->k_R_payload;
  k_Omega_0_= cmd->k_Omega_payload;
  k_q_      = cmd->k_cable_angle;
  k_w_      = cmd->k_cable_angular_velocity;

  publishSO3Command();
}

void ControlCableVisionNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  have_odom_ = true;

  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  current_orientation_ = Eigen::Quaternionf(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

//  controller_.setPosition(position);
//  controller_.setVelocity(velocity);
//  controller_.setCurrentOrientation(current_orientation_);

  if(position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some time
    if(!position_cmd_updated_)
      publishSO3Command();
    position_cmd_updated_ = false;
  }
}

void ControlCableVisionNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
  // Reset integral when toggling motor state
//  controller_.resetIntegrals();
}

void ControlCableVisionNodelet::corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void ControlCableVisionNodelet::onInit()
{
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

//  n.param("mass", mass_, 0.5f);
//  controller_.setMass(mass_);
//  controller_.setGravity(g_);

//  n.param("use_external_yaw", use_external_yaw_, true);

//  n.param("gains/ki/x", ki_[0], 0.0f);
//  n.param("gains/ki/y", ki_[1], 0.0f);
//  n.param("gains/ki/z", ki_[2], 0.0f);

//  n.param("gains/kib/x", kib_[0], 0.0f);
//  n.param("gains/kib/y", kib_[1], 0.0f);
//  n.param("gains/kib/z", kib_[2], 0.0f);

//  n.param("gains/rot/x", kR_[0], 1.5f);
//  n.param("gains/rot/y", kR_[1], 1.5f);
//  n.param("gains/rot/z", kR_[2], 1.0f);
//  n.param("gains/ang/x", kOm_[0], 0.13f);
//  n.param("gains/ang/y", kOm_[1], 0.13f);
//  n.param("gains/ang/z", kOm_[2], 0.1f);

//  n.param("corrections/kf", corrections_[0], 0.0f);
//  n.param("corrections/r", corrections_[1], 0.0f);
//  n.param("corrections/p", corrections_[2], 0.0f);

//  float max_pos_int, max_pos_int_b;
//  n.param("max_pos_int", max_pos_int, 0.5f);
//  n.param("mas_pos_int_b", max_pos_int_b, 0.5f);
//  controller_.setMaxIntegral(max_pos_int);
//  controller_.setMaxIntegralBody(max_pos_int_b);

//  float max_tilt_angle;
//  n.param("max_tilt_angle", max_tilt_angle, static_cast<float>(M_PI));
//  controller_.setMaxTiltAngle(max_tilt_angle);

  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  odom_sub_ = n.subscribe("odom", 10, &ControlCableVisionNodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
//  position_cmd_sub_ = n.subscribe("position_cmd", 10, &ControlCableVisionNodelet::position_cmd_callback, this,
//                                  ros::TransportHints().tcpNoDelay());
  enable_motors_sub_ = n.subscribe("motors", 2, &ControlCableVisionNodelet::enable_motors_callback, this,
                                   ros::TransportHints().tcpNoDelay());
  corrections_sub_ = n.subscribe("corrections", 10, &ControlCableVisionNodelet::corrections_callback, this,
                                 ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ControlCableVisionNodelet, nodelet::Nodelet);
