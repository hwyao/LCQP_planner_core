#include <franka_controllers/LCQP_controller_quickconcept.hpp>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#define SCENE_DYNAMIC
//#undef SCENE_DYNAMIC
#define SCENE_STATIC
#undef SCENE_STATIC

using namespace Eigen;


namespace franka_controllers {

bool LCQPControllerQuickConcept::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  ROS_INFO("LCQPControllerQuickConcept: LCQP controller init");

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("LCQPControllerQuickConcept: Could not read parameter arm_id");
    return false;
  }
                                            
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse joint names");
    return false;
  }

  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("LCQPControllerQuickConcept: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "LCQPControllerQuickConcept: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "LCQPControllerQuickConcept: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "LCQPControllerQuickConcept: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "LCQPControllerQuickConcept: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "LCQPControllerQuickConcept: Error getting position joint interface from hardware!");
    return false;
  }

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "LCQPControllerQuickConcept: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(velocity_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "LCQPControllerQuickConcept: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  // load parameters
  if(!node_handle.getParam("goal", goal_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse goal");
  }
  if(!node_handle.getParam("constMainTask", constMainTask_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse constMainTask");
  }
  if(!node_handle.getParam("constContactTask", constContactTask_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse constContactTask");
  }
  if(!node_handle.getParam("constError", constError_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse constError");
  }
  if(!node_handle.getParam("solverBound", solverBound_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse solverBound");
  }
  if(!node_handle.getParam("safetyDistance", safetyDistance_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse safetyDistance");
  }
  if(!node_handle.getParam("safetyIgnore", safetyIgnore_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse safetyIgnore");
  }
  if(!node_handle.getParam("linkObstacleAvoid", linkObstacleAvoid_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse linkObstacleAvoid");
  }
  if(!node_handle.getParam("robustJinvLambda", robustJinvLambda_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse robustJinvLambda");
  }
  if(!node_handle.getParam("tolerance", tolerance_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse tolerance");
  }
  if(!node_handle.getParam("maximumTime", maximumTime_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse maximumTime");
  }
  if(!node_handle.getParam("jacobianSkipcount", jacobianSkipcount_)){
    ROS_ERROR("LCQPControllerQuickConcept: Could not parse jacobianSkipcount");
  }
  
//   if(!node_handle.getParam("LowPassJoint", LowPassJoint_)){
//     ROS_ERROR("LCQPControllerQuickConcept: Could not parse LowPass");
//   }

  /// initialize the obstacles and radius
  #ifdef SCENE_DYNAMIC
    obsList_.push_back(Vector3d(0.35, 0.5, 0.4)); 
    obsRadiusList_.push_back(0.05);
    obsList_.push_back(Vector3d(-0.13, -0.52, 0.5)); 
    obsRadiusList_.push_back(0.1);
    obsList_.push_back(Vector3d(0.25, 0.3, 0.4)); 
    obsRadiusList_.push_back(0.05);
    obsList_.push_back(Vector3d(0.30, 0.3, 0.4));
    obsRadiusList_.push_back(0.05); 
    //obsList_.push_back(Vector3d(0.4, 0.6, 0.3));
    //obsRadiusList_.push_back(0.05); 
    // obsList_.push_back(Vector3d(0.30, 0.4, 0.4));
    // obsRadiusList_.push_back(0.05); 
  #endif

  #ifdef SCENE_STATIC
    obsList_.push_back(Vector3d(0.4, 0.35, 0.35)); 
    obsRadiusList_.push_back(0.05);
    obsList_.push_back(Vector3d(0.3, 0.30, 0.75)); 
    obsRadiusList_.push_back(0.1);
    // obsList_.push_back(Vector3d(0.5, 0.00, 0.75)); 
    // obsRadiusList_.push_back(0.05);
    obsList_.push_back(Vector3d(0.5, 0.10, 0.75));
    obsRadiusList_.push_back(0.05); 
    obsList_.push_back(Vector3d(0.5, -0.25, 0.45));
    obsRadiusList_.push_back(0.1); 
    // obsList_.push_back(Vector3d(0.1, 0.33, 0.70));
    // obsRadiusList_.push_back(0.1); 
    obsList_.push_back(Vector3d(0.4, 0.35, 0.45));
    obsRadiusList_.push_back(0.05); 
    goal_[0] = 0.65;
    goal_[1] = 0.35;
    goal_[2] = 0.25;
  #endif

  #ifdef SCENE_DYNAMIC

  #endif


  controller_ = LCQP_controller::Controller();
  frankaModel_ = LCQP_controller::FrankaModel(state_interface->getHandle(arm_id + "_robot"), model_interface->getHandle(arm_id + "_model"));
  controller_.constMainTask_ = constMainTask_;
  controller_.constContactTask_ = constContactTask_;
  controller_.constError_ = constError_;
  controller_.solverBound_ = solverBound_;
  controller_.safetyDistance_ = safetyDistance_;
  controller_.safetyIgnore_ = safetyIgnore_;
  controller_.linkObstacleAvoid_ = linkObstacleAvoid_;
  controller_.robustJinvLambda_ = robustJinvLambda_;
  controller_.tolerance_ = tolerance_;
  controller_.jacobianSkipcount_ = jacobianSkipcount_;
  controller_.initPlanner(goal_, frankaModel_, obsList_.size(), linkObstacleAvoid_.size(), obsList_, obsRadiusList_);

  ROS_INFO("LCQPControllerQuickConcept: LCQP controller initialized");

  // Publisher for the obstacles in Rviz
  obstacle_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

  distance_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("distance", 1);

  traj_pub_ = node_handle.advertise<nav_msgs::Path>("trajectory_planned", 1);

  vel_pub_ = node_handle.advertise<geometry_msgs::TwistStamped>("velocity_command_planned", 1);
  vel_pub_2 = node_handle.advertise<geometry_msgs::TwistStamped>("velocity_command_LCQP", 1);

  return true;
}

void LCQPControllerQuickConcept::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}


void LCQPControllerQuickConcept::update(const ros::Time& /*time*/, const ros::Duration& period) {
  //ROS_INFO("LCQPControllerQuickConcept: Current time: start");
  elapsed_time_ += period;

  VectorXd qDot;

  qDot = controller_.nextStep(obsList_, obsRadiusList_);

  std_msgs::Float64MultiArray distance;
  
  // only publish each 1/30 seconds
  do
  {
    static ros::Time last_pub_time = ros::Time(0);
    if (ros::Time::now() - last_pub_time < ros::Duration(1.0 / 30.0))
    {
      break;
    }
    last_pub_time = ros::Time::now();
    distance.data.resize(controller_.contactDistMtx.size());
    for (size_t i = 0; i < controller_.contactDistMtx.size(); ++i) {
      distance.data[i] = controller_.contactDistMtx(i);
    }
    distance_pub_.publish(distance);
    publishObstacle(obsList_, obsRadiusList_);
    
    MatrixXd EE_frame = frankaModel_.getTransformation(VectorXd::Zero(7), 7);
    Vector3d position = EE_frame.block(0, 3, 3, 1);
    Vector3d goal = Vector3d(Map<Vector3d>(goal_.data()));

    int num_points = 10;
    std::vector<Vector3d> path(num_points);

    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        path[i] = (1.0 - t) * position + t * goal;
    }

    VectorXd q = frankaModel_.getJointState();
    MatrixXd J = frankaModel_.getJacobian(q, 7);
    Vector3d vel_LCQP = (J * qDot).block(0, 0, 3, 1);
    vel_LCQP = vel_LCQP / vel_LCQP.norm();
    vel_LCQP = EE_frame.block(0,0,3,3) * vel_LCQP;

    Vector3d vel_cmd = goal - position;
    vel_cmd = vel_cmd / vel_cmd.norm();
    vel_cmd = EE_frame.block(0,0,3,3) * vel_cmd;

    publishTrajectory(path, vel_cmd, vel_LCQP);
  } while (false);
  

  #ifdef SCENE_DYNAMIC
  static int phase = 1;
  if (phase == 1){
    obsList_[1] = obsList_[1] + Vector3d(0, 0.10 * 0.001, 0);
    if (obsList_[1](1) > - 0.2){
        phase = 2;
    }
  }
  else{
    obsList_[1] = obsList_[1] + Vector3d(0, -0.05 * 0.001, 0);
    obsList_[2] = obsList_[2] + Vector3d(0, -0.05 * 0.001, 0);
    obsList_[0] = obsList_[0] + Vector3d(0, -0.05 * 0.001, 0);
    obsList_[3] = obsList_[3] + Vector3d(0, -0.05 * 0.001, 0);
    //obsList_[4] = obsList_[4] + Vector3d(0, -0.05 * 0.001, 0);
    //obsList_[5] = obsList_[5] + Vector3d(0, -0.05 * 0.001, 0);
  }
  #endif

  // filter the qDot (is disabled by setting LowPass_ = 0. But anyway we keep this here.)
  //   static VectorXd qDotFiltered = qDot;
  //   double alpha = LowPassJoint_ * 0.001;
  //   qDotFiltered = qDot + alpha * (qDotFiltered - qDot);

  Vector3d position = frankaModel_.getTransformation(VectorXd::Zero(7), 7).block(0, 3, 3, 1);
  Vector3d difference = position - Vector3d(Map<Vector3d>(goal_.data()));
  if (difference.norm() < tolerance_){
    ROS_INFO_ONCE("LCQPControllerQuickConcept: Goal reached with tolerance %f", tolerance_);
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(0);
    }
  }
  else{
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(qDot(i));
    }
  }
  //ROS_INFO("LCQPControllerQuickConcept: Current time: finished");
}

void LCQPControllerQuickConcept::publishTrajectory(const std::vector<Vector3d>& trajList, const Vector3d& velcmd, const Vector3d& velLCQP){
    nav_msgs::Path path_msg;
    geometry_msgs::TwistStamped vel_msg;
    geometry_msgs::TwistStamped vel_LCQP_msg;

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "panda_link0";
    vel_msg.header.stamp = ros::Time::now();
    vel_msg.header.frame_id = "panda_link8"; 
    vel_LCQP_msg.header.stamp = ros::Time::now();
    vel_LCQP_msg.header.frame_id = "panda_link8";

    for (const auto& point : trajList) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "traj_planned";

        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();

        path_msg.poses.push_back(pose);
    }
    vel_msg.twist.linear.x = velcmd.x();
    vel_msg.twist.linear.y = velcmd.y();
    vel_msg.twist.linear.z = velcmd.z();

    vel_LCQP_msg.twist.linear.x = velLCQP.x();
    vel_LCQP_msg.twist.linear.y = velLCQP.y();
    vel_LCQP_msg.twist.linear.z = velLCQP.z();

    traj_pub_.publish(path_msg);
    vel_pub_.publish(vel_msg);
    vel_pub_2.publish(vel_LCQP_msg);
}

void LCQPControllerQuickConcept::publishObstacle(const std::vector<Vector3d>& obsList, const std::vector<double>& obsRadiusList){
    visualization_msgs::MarkerArray markerArray;
    for (size_t i = 0; i < obsList.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.ns = "obstacles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = obsList[i](0);
        marker.pose.position.y = obsList[i](1);
        marker.pose.position.z = obsList[i](2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obsRadiusList[i] * 2;
        marker.scale.y = obsRadiusList[i] * 2;
        marker.scale.z = obsRadiusList[i] * 2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        markerArray.markers.push_back(marker);
    }
    obstacle_pub_.publish(markerArray);
}


}  // namespace franka_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::LCQPControllerQuickConcept,
                       controller_interface::ControllerBase)