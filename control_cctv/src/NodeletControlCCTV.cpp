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
      have_odom_(false),
      g_(9.81),
      current_orientation_(Eigen::Quaterniond::Identity())
  {
//    controller_.resetIntegrals();
  }

  void onInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishSO3Command(); // Still publish so3commands to attitude controller
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  void payload_position_cmd_callback(const msgs_cctv::PayloadCommandConstPtr &cmd);
  void quad_odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);

  // Keep two controllers to switch between them
  ControlCCTV cctv_controller_;
  SO3Control so3_controller_;

  ros::Publisher so3_command_pub_;
  ros::Subscriber payload_odom_sub_;
  ros::Subscriber quad_odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  //Robot desired state (so3 controller)
  Eigen::Vector3d des_pos_, des_vel_, des_acc_, des_jrk_;
  double des_yaw_, des_yaw_dot_;

  //SO3 controller gains
  Eigen::Vector3d kx_, kv_, ki_, kib_;

  //Payload desired state (cctv controller)
  Eigen::Vector3d des_pos_0_, des_vel_0_, des_acc_0_;
  Eigen::Matrix3d des_R_0_;
  Eigen::Vector3d des_Omega_0_, des_alpha_0_;

  double k_pos_0_, k_vel_0_, k_R_0_, k_Omega_0_, k_q_, k_w_;
  double des_yaw_i_, des_yaw_dot_i_;
  double current_yaw_;
  bool enable_motors_, use_external_yaw_, have_odom_;
  double kR_[3], kOm_[3], corrections_[3];

  //System definition
  const double g_;
  double robot_mass_;
  double payload_mass_;
  Eigen::Matrix3d J_0_;
  Eigen::Matrix3d J_i_;
  double cable_length_;
  Eigen::Matrix3d rho_;
  int idx_;
  int n_bots_;

  Eigen::Quaterniond current_orientation_;

  enum Controller {SO3_CONTROL, CCTV_CONTROL};
  Controller active_controller_ = SO3_CONTROL;
};


void NodeletControlCCTV::publishSO3Command()
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

  bool DB_CTRL_SWITCH = 1;

  quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command);
  so3_command->header.stamp = ros::Time::now();
  so3_command->header.frame_id = frame_id_;

  switch (active_controller_) {

  case SO3_CONTROL: {
    ROS_INFO_COND(DB_CTRL_SWITCH, "NodeletControlCCTV - Using so3 controller to publish s03 command");
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

    break;
  }
  case CCTV_CONTROL: {
    ROS_DEBUG_COND(DB_CTRL_SWITCH, "NodeletControlCCTV - Using cctv controller to publish s03 command");
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

void NodeletControlCCTV::payload_position_cmd_callback(const msgs_cctv::PayloadCommandConstPtr &cmd)
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
  //position_cmd_init_ = true;

  publishSO3Command();
}

void NodeletControlCCTV::quad_odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  have_odom_ = true;

  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  current_orientation_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  so3_controller_.setPosition(position);
  so3_controller_.setVelocity(velocity);
  so3_controller_.setCurrentOrientation(current_orientation_);

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
  priv_nh.param("mass", robot_mass_, 0.5);
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

  // Set cctv controller params
  cctv_controller_.set_m_0(payload_mass_);
  cctv_controller_.set_J_0(Eigen::Matrix3d::Identity());
  cctv_controller_.set_m_i(robot_mass_);
  cctv_controller_.set_J_i(Eigen::Matrix3d::Identity());
  cctv_controller_.set_l_i(cable_length_);
  cctv_controller_.set_rho(rho_);
  cctv_controller_.set_g  (g_);
  cctv_controller_.set_idx(0);
  cctv_controller_.set_n_bots(3);



  so3_command_pub_    = priv_nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  payload_odom_sub_   = priv_nh.subscribe("payload_odom", 10, &NodeletControlCCTV::payload_position_cmd_callback, this, ros::TransportHints().tcpNoDelay());
  quad_odom_sub_      = priv_nh.subscribe("quad_odom",    10, &NodeletControlCCTV::quad_odom_callback,            this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_   = priv_nh.subscribe("position_cmd", 10, &NodeletControlCCTV::position_cmd_callback,         this, ros::TransportHints().tcpNoDelay());
  enable_motors_sub_  = priv_nh.subscribe("motors",       2,  &NodeletControlCCTV::enable_motors_callback,        this, ros::TransportHints().tcpNoDelay());
  corrections_sub_    = priv_nh.subscribe("corrections",  10, &NodeletControlCCTV::corrections_callback,           this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("nodelet control cctv is alive");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NodeletControlCCTV, nodelet::Nodelet);
