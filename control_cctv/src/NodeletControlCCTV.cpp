#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <msgs_cctv/PayloadCommand.h>
#include <quadrotor_msgs/Corrections.h>
#include <std_srvs/SetBool.h>
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
  void payload_cmd_callback (const msgs_cctv::PayloadCommand::ConstPtr &payload_cmd);
  void payload_odom_callback(const nav_msgs::Odometry::ConstPtr &pl_odom);
  void quad_odom_callback(const nav_msgs::Odometry::ConstPtr &quad_odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr &msg);

  bool use_cctv_control_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  void estimate_cable_state(void);
  void pub_viz(void);
  void pub_vec3(ros::Publisher pub, Eigen::Vector3d vec);

  // Keep two controllers to switch between them
  ControlCCTV cctv_controller_;
  SO3Control so3_controller_;

  ros::Publisher so3_command_pub_;
  ros::Subscriber payload_odom_sub_;
  ros::Subscriber quad_odom_sub_, position_cmd_sub_, enable_motors_sub_, corrections_sub_;

  // Service to switch controller
  ros::ServiceServer use_cctv_control_srv_;

  // Debug publishers
  ros::Publisher q_i_pub;
  ros::Publisher q_i_dot_pub;
  ros::Publisher w_i_pub;

  ros::Publisher q_i_raw_pub;
  ros::Publisher q_i_dot_raw_pub;
  ros::Publisher w_i_raw_pub;

  ros::Publisher u_i_pub, u_i_prl_pub, u_i_prp_pub;

  ros::Publisher e_pos_0_pub;
  ros::Publisher e_vel_0_pub;
  ros::Publisher e_R_0_pub;
  ros::Publisher e_Omega_0_pub;
  ros::Publisher e_q_i_pub;
  ros::Publisher e_w_i_pub;

  ros::Publisher F_0_des_pub;
  ros::Publisher M_0_des_pub;
  ros::Publisher q_i_des_pub;

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

  Eigen::Vector3d kp_pos_0_;
  Eigen::Vector3d ki_pos_0_;
  double max_pos_0_int;
  Eigen::Vector3d kd_pos_0_;

  Eigen::Vector3d k_R_0_;
  Eigen::Vector3d k_Omega_0_;

  double kp_q_, kd_q_;
  double ki_q_, max_e_q_int;
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
  Eigen::Vector3d     q_i_;     // unit vector from robot to attachment point
  Eigen::Vector3d     q_i_dot_;
  Eigen::Vector3d     w_i_;

  Eigen::Vector3d   q_i_raw;
  Eigen::Vector3d   q_i_dot_raw;
  Eigen::Vector3d   w_i_raw;


  // Payload state
  Eigen::Vector3d     pos_0_;
  Eigen::Vector3d     vel_0_;
  Eigen::Vector3d     pl_attach_;
  Eigen::Quaterniond  R_0_;
  Eigen::Vector3d     Omega_0_;

  // Filter constants
  double tau_q_i;
  double tau_q_i_dot;
  double tau_w_i;

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

  // bool DB_CTRL_SWITCH = 1;

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
                               kp_pos_0_,
                               ki_pos_0_,
                               kd_pos_0_,
                               k_R_0_,
                               k_Omega_0_,
                               kp_q_,
                               kd_q_,
                               ki_q_);

//    viz_cctv_control();

    break;
  }
  case CCTV_CONTROL: {
    ROS_INFO_THROTTLE(1, "NodeletControlCCTV - Using cctv controller to publish s03 command");
    cctv_controller_.calculateControl(des_pos_0_,
                               des_vel_0_,
                               des_acc_0_,
                               des_R_0_,
                               des_Omega_0_,
                               des_alpha_0_,
                               des_yaw_i_,
                               kp_pos_0_,
                               ki_pos_0_,
                               kd_pos_0_,
                               k_R_0_,
                               k_Omega_0_,
                               kp_q_,
                               kd_q_,
                               ki_q_);

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

void NodeletControlCCTV::payload_odom_callback(const nav_msgs::Odometry::ConstPtr &pl_odom)
{
  pos_0_(0)  = pl_odom->pose.pose.position.x;
  pos_0_(1)  = pl_odom->pose.pose.position.y;
  pos_0_(2)  = pl_odom->pose.pose.position.z;
  R_0_.x() = pl_odom->pose.pose.orientation.x;
  R_0_.y() = pl_odom->pose.pose.orientation.y;
  R_0_.z() = pl_odom->pose.pose.orientation.z;
  R_0_.w() = pl_odom->pose.pose.orientation.w;
  // R_0_.setIdentity();
  Omega_0_(0)  = pl_odom->twist.twist.angular.x;
  Omega_0_(1)  = pl_odom->twist.twist.angular.y;
  Omega_0_(2)  = pl_odom->twist.twist.angular.z;
  vel_0_(0)  = pl_odom->twist.twist.linear.x;
  vel_0_(1)  = pl_odom->twist.twist.linear.y;
  vel_0_(2)  = pl_odom->twist.twist.linear.z;


  // Set payload state using controller accessors
  cctv_controller_.set_pos_0  (pos_0_);
  cctv_controller_.set_vel_0  (vel_0_);
  cctv_controller_.set_R_0    (R_0_.normalized().toRotationMatrix());
  cctv_controller_.set_Omega_0(Omega_0_);
  estimate_cable_state();

  // publishSO3Command();
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
  // publishSO3Command();
}

//------------------------------------------------------------------------------------------------
// Hacky stuff about cable angle, todo NEEDS TO BE DONE BETTER
//------------------------------------------------------------------------------------------------
void NodeletControlCCTV::estimate_cable_state(void){

  // Note: p = location, v = velocity, p_b_0 = position, of(sub) b, in-frame(super) 0

  // 1. calculate q_i
  const Eigen::Matrix3d R_pl_0 = R_0_.normalized().toRotationMatrix();
  const Eigen::Vector3d p_attach_0 = pos_0_ + (R_pl_0 * rho_i_);
  pl_attach_ = p_attach_0;
  const Eigen::Vector3d p_attach_quad = (p_attach_0 - quad_pos_);
  if(p_attach_quad.norm() > 0.01){
    q_i_raw = p_attach_quad.normalized();
  } else {
    q_i_raw = q_i_;
  }
  double alpha_q_i      = dt / (tau_q_i + dt);
  q_i_      = q_i_      + alpha_q_i     * (q_i_raw      - q_i_);

  // 2. calculate q_i_dot
  const Eigen::Vector3d v_attach_0 = vel_0_ + Omega_0_.cross(rho_i_);
  const Eigen::Vector3d v_attach_quad = v_attach_0 - quad_vel_;
  q_i_dot_raw = v_attach_quad / cable_length_;

  // 3. calculate w_i (yikes)
  w_i_raw = (q_i_.cross(v_attach_quad)) / (cable_length_);
  double alpha_w_i      = dt / (tau_w_i + dt);
  w_i_      = w_i_      + alpha_w_i     * (w_i_raw      - w_i_);

  // Low pass filter all three
  static ros::Time t_last = ros::Time(0);
  ros::Time t_now = ros::Time::now();
  double dt = (t_now - t_last).toSec();
  t_last = t_now;

  double alpha_q_i_dot  = dt / (tau_q_i_dot + dt);

  // ROS_WARN("alpha_q_i is: %2.4f", alpha_q_i);

  q_i_dot_  = q_i_dot_  + alpha_q_i_dot * (q_i_dot_raw  - q_i_dot_);
  q_i_.normalize();

  for (int i =0; i<3; i++){
    if(std::isnan(q_i_(i)))     q_i_      = -Eigen::Vector3d::UnitZ();
    if(std::isnan(q_i_dot_(i))) q_i_dot_  = Eigen::Vector3d::Zero();
    if(std::isnan(w_i_(i)))     w_i_      = Eigen::Vector3d::Zero();
  }


  cctv_controller_.set_q_i(q_i_);
  cctv_controller_.set_q_i_dot(q_i_dot_);
  cctv_controller_.set_w_i(w_i_);

  pub_viz();
}

void NodeletControlCCTV::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
  if(active_controller_ == SO3_CONTROL){
    ROS_INFO_THROTTLE(1, "Control nodelet is using SO3 control and recieved POSITION command");
  } else if (active_controller_ == CCTV_CONTROL){
    ROS_WARN_THROTTLE(1, "Control nodelet is using CCTV control and recieved PAYLOAD command");
  }
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

void NodeletControlCCTV::payload_cmd_callback(const msgs_cctv::PayloadCommand::ConstPtr &payload_cmd)
{
  if(active_controller_ == SO3_CONTROL){
    ROS_WARN_THROTTLE(1, "Control nodelet is using SO3 control and recieved PAYLOAD command");
  } else if (active_controller_ == CCTV_CONTROL){
    ROS_INFO_THROTTLE(1, "Control nodelet is using CCTV control and recieved POSITION command");
  }

  // Extract linear componenets
  des_pos_0_(0) = payload_cmd->pos.position.x;
  des_pos_0_(1) = payload_cmd->pos.position.y;
  des_pos_0_(2) = payload_cmd->pos.position.z;

  des_vel_0_(0) = payload_cmd->vel.linear.x;
  des_vel_0_(1) = payload_cmd->vel.linear.y;
  des_vel_0_(2) = payload_cmd->vel.linear.z;

  des_acc_0_(0) = payload_cmd->acc.linear.x;
  des_acc_0_(1) = payload_cmd->acc.linear.y;
  des_acc_0_(2) = payload_cmd->acc.linear.z;

  // Extract angular components
  Eigen::Quaterniond cmd_quaternion = Eigen::Quaterniond(payload_cmd->pos.orientation.x,
                                      payload_cmd->pos.orientation.y,
                                      payload_cmd->pos.orientation.z,
                                      payload_cmd->pos.orientation.w);
  des_R_0_ = cmd_quaternion.normalized().toRotationMatrix();

  des_Omega_0_(0) = payload_cmd->vel.angular.x;
  des_Omega_0_(1) = payload_cmd->vel.angular.y;
  des_Omega_0_(2) = payload_cmd->vel.angular.z;

  des_alpha_0_(0) = payload_cmd->acc.angular.x;
  des_alpha_0_(1) = payload_cmd->acc.angular.y;
  des_alpha_0_(2) = payload_cmd->acc.angular.z;
}


void NodeletControlCCTV::enable_motors_callback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data)
    ROS_INFO("Control cctv Enabling motors");
  else
    ROS_INFO("control cctv Disabling motors");

  enable_motors_ = msg->data;

  // Reset integral when toggling motor state
  so3_controller_.resetIntegrals();
  cctv_controller_.resetIntegrals();
}

bool NodeletControlCCTV::use_cctv_control_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  ROS_INFO("switching controller");
  if(req.data){
    ROS_INFO("Switching to cctv control... good luck");
    active_controller_ = CCTV_CONTROL;
  } else {
    ROS_INFO("Switching so3 control");
    active_controller_ = SO3_CONTROL;
  }

  // Reset integral when toggling motor state
  so3_controller_.resetIntegrals();
  cctv_controller_.resetIntegrals();
  res.success = true;
  res.message = "did the switch";
  return true;
}

void NodeletControlCCTV::pub_viz()
{
  pub_vec3(q_i_pub,         q_i_);
  pub_vec3(q_i_dot_pub,     q_i_dot_);
  pub_vec3(w_i_pub,         w_i_);

  pub_vec3(q_i_raw_pub,         q_i_raw);
  pub_vec3(q_i_dot_raw_pub,     q_i_dot_raw);
  pub_vec3(w_i_raw_pub,         w_i_raw);

  pub_vec3(u_i_pub,         cctv_controller_.get_u_i());
  pub_vec3(u_i_prl_pub,     cctv_controller_.get_u_i_prl());
  pub_vec3(u_i_prp_pub,     cctv_controller_.get_u_i_prp());

  pub_vec3(e_pos_0_pub,     cctv_controller_.e_pos_0);
  pub_vec3(e_vel_0_pub,     cctv_controller_.e_vel_0);
  pub_vec3(e_R_0_pub,       cctv_controller_.e_R_0);
  pub_vec3(e_Omega_0_pub,   cctv_controller_.e_Omega_0);
  pub_vec3(e_q_i_pub,       cctv_controller_.e_q_i);
  pub_vec3(e_w_i_pub,       cctv_controller_.e_w_i);

  pub_vec3(F_0_des_pub,     cctv_controller_.F_0_des);
  pub_vec3(M_0_des_pub,     cctv_controller_.M_0_des);
  pub_vec3(q_i_des_pub,     cctv_controller_.q_i_des);
}

void NodeletControlCCTV::pub_vec3(ros::Publisher pub, Eigen::Vector3d vec)
{
  geometry_msgs::Vector3Stamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.vector.x = vec(0);
  msg.vector.y = vec(1);
  msg.vector.z = vec(2);
  pub.publish(msg);
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
  priv_nh.param("payload_mass", payload_mass_, 0.1);
  priv_nh.param("cable_length", cable_length_, 0.33);
  priv_nh.param("number_of_robots", n_bots_, 3);

  std::string my_name;
  priv_nh.param("my_name", my_name, std::string("penis"));

  std::vector<std::string> robot_names;
  if(!priv_nh.getParam("quad_names", robot_names)) ROS_ERROR("Control CCTV Params: Could not load robot_names");

  idx_ = 0;
  for(unsigned int i=0; i<robot_names.size(); i++){
    if(!my_name.compare(robot_names[i])) idx_ = i;
  }
  ROS_INFO("Control CCTV Params (my_name): %s", my_name.c_str());
  ROS_INFO("Control CCTV Params (idx_): %i", idx_);

  // Get Payload gains
  priv_nh.param("kp_pos_0/x", kp_pos_0_(0),      0.0);
  priv_nh.param("kp_pos_0/y", kp_pos_0_(1),      0.0);
  priv_nh.param("kp_pos_0/z", kp_pos_0_(2),      0.0);

  priv_nh.param("ki_pos_0/x", ki_pos_0_(0),    0.0);
  priv_nh.param("ki_pos_0/y", ki_pos_0_(1),    0.0);
  priv_nh.param("ki_pos_0/z", ki_pos_0_(2),    0.0);
  priv_nh.param("max_pos_0_int", max_pos_0_int,      0.0);
  
  priv_nh.param("kd_pos_0/x", kd_pos_0_(0),     0.0);
  priv_nh.param("kd_pos_0/y", kd_pos_0_(1),     0.0);
  priv_nh.param("kd_pos_0/z", kd_pos_0_(2),     0.0);

  priv_nh.param("k_R_0/x", k_R_0_(0),         0.0);
  priv_nh.param("k_R_0/y", k_R_0_(1),         0.0);
  priv_nh.param("k_R_0/z", k_R_0_(2),         0.0);

  priv_nh.param("k_Omega_0/x", k_Omega_0_(0), 0.0);
  priv_nh.param("k_Omega_0/y", k_Omega_0_(1), 0.0);
  priv_nh.param("k_Omega_0/z", k_Omega_0_(2), 0.0);

  priv_nh.param("kp_q", kp_q_,              0.0);
  priv_nh.param("kd_q", kd_q_,              0.0);
  priv_nh.param("ki_q", ki_q_,              0.0);

  priv_nh.param("max_e_q_int", max_e_q_int,  0.0);

  priv_nh.param("tau_q_i",      tau_q_i,       0.05);
  priv_nh.param("tau_q_i_dot",  tau_q_i_dot,   0.05);
  priv_nh.param("tau_w_i",      tau_w_i,       0.05);

  // Retrieve attachment point params for all robots
  for (int i=0; i<n_bots_; i++){
    std::string param_name = "attachment/rho_" + std::to_string(i);
    if(!priv_nh.getParam(param_name + "/x", rho_(0, i)))
      ROS_ERROR("Control CCTV Params: Could not find param %s/x", param_name.c_str());
    if(!priv_nh.getParam(param_name + "/y", rho_(1, i)))
      ROS_ERROR("Control CCTV Params: Could not find param %s/y", param_name.c_str());
    if(!priv_nh.getParam(param_name + "/z", rho_(2, i)))
      ROS_ERROR("Control CCTV Params: Could not find param %s/z", param_name.c_str());
  }
  rho_i_ = rho_.block<3,1>(0, idx_);

  // Load Payload Inertia Matrix
  std::vector<double> J_0_param;
  if(!priv_nh.getParam("J_0", J_0_param)) ROS_WARN("Control CCTV Params: Could not load J_0_");
  for(int i=0; i<3; i++){
     J_0_(i,i) = J_0_param[i];
  }

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
  cctv_controller_.set_max_e_q_int (max_e_q_int);

  // Set cctv_controller des to zero TODO not this

  des_pos_0_.setZero();
  des_vel_0_.setZero();
  des_acc_0_.setZero();
  des_R_0_.setIdentity();
  des_Omega_0_.setZero();
  des_alpha_0_.setZero();

  so3_command_pub_  = priv_nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  q_i_pub           = priv_nh.advertise<geometry_msgs::Vector3Stamped>("q_i", 10);
  q_i_dot_pub       = priv_nh.advertise<geometry_msgs::Vector3Stamped>("q_i_dot", 10);
  w_i_pub           = priv_nh.advertise<geometry_msgs::Vector3Stamped>("w_i", 10);

  q_i_raw_pub           = priv_nh.advertise<geometry_msgs::Vector3Stamped>("q_i_raw", 10);
  q_i_dot_raw_pub       = priv_nh.advertise<geometry_msgs::Vector3Stamped>("q_i_dot_raw", 10);
  w_i_raw_pub           = priv_nh.advertise<geometry_msgs::Vector3Stamped>("w_i_raw", 10);

  // controller debugging publishers
  e_pos_0_pub   = priv_nh.advertise<geometry_msgs::Vector3Stamped>("e_pos_0", 10);
  e_vel_0_pub   = priv_nh.advertise<geometry_msgs::Vector3Stamped>("e_vel_0", 10);
  e_R_0_pub     = priv_nh.advertise<geometry_msgs::Vector3Stamped>("e_R_0", 10);
  e_Omega_0_pub = priv_nh.advertise<geometry_msgs::Vector3Stamped>("e_Omega_0", 10);
  e_q_i_pub     = priv_nh.advertise<geometry_msgs::Vector3Stamped>("e_q_i", 10);
  e_w_i_pub     = priv_nh.advertise<geometry_msgs::Vector3Stamped>("e_w_i", 10);

  F_0_des_pub   = priv_nh.advertise<geometry_msgs::Vector3Stamped>("F_0_des", 10);
  M_0_des_pub   = priv_nh.advertise<geometry_msgs::Vector3Stamped>("M_0_des", 10);
  q_i_des_pub   = priv_nh.advertise<geometry_msgs::Vector3Stamped>("q_i_des", 10);

  u_i_pub       = priv_nh.advertise<geometry_msgs::Vector3Stamped>("u_i", 10);
  u_i_prl_pub   = priv_nh.advertise<geometry_msgs::Vector3Stamped>("u_i_prl", 10);
  u_i_prp_pub   = priv_nh.advertise<geometry_msgs::Vector3Stamped>("u_i_prp", 10);

  // Useful subscribers
  payload_odom_sub_   = priv_nh.subscribe("payload_odom", 10, &NodeletControlCCTV::payload_odom_callback,         this, ros::TransportHints().tcpNoDelay());
  payload_cmd_sub_    = priv_nh.subscribe("payload_cmd",  10, &NodeletControlCCTV::payload_cmd_callback,          this, ros::TransportHints().tcpNoDelay());
  quad_odom_sub_      = priv_nh.subscribe("quad_odom",    10, &NodeletControlCCTV::quad_odom_callback,            this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_   = priv_nh.subscribe("position_cmd", 10, &NodeletControlCCTV::position_cmd_callback,         this, ros::TransportHints().tcpNoDelay());
  enable_motors_sub_  = priv_nh.subscribe("motors",       2,  &NodeletControlCCTV::enable_motors_callback,        this, ros::TransportHints().tcpNoDelay());
  corrections_sub_    = priv_nh.subscribe("corrections",  10, &NodeletControlCCTV::corrections_callback,          this, ros::TransportHints().tcpNoDelay());

  use_cctv_control_srv_ = priv_nh.advertiseService("use_cctv_controller", &NodeletControlCCTV::use_cctv_control_callback, this);

  ROS_INFO("nodelet control cctv is alive");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NodeletControlCCTV, nodelet::Nodelet);
