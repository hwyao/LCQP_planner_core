#pragma once

#include <memory>
#include <mutex>
#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

#include <LCQProblem.hpp>
#include <LCQP_controller/LCQP_controller.hpp>
#include <LCQP_controller/FrankaModel.hpp>

using namespace Eigen;

namespace franka_controllers {

class LCQPControllerQuickConcept : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface,
                                           franka_hw::FrankaModelInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_; //position_joint_interface_
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  ros::Duration elapsed_time_;

  std::vector<double> goal_ = {0.1, 0.450, 0.5}; /// Goal of the planner. Here we accept the goal as position.
  double constMainTask_ = 0.1;                   /// constant for main task c*J*dx
  double constContactTask_ = 0.006;              /// constant for collision avoidance h*J1_1*N1_1*qdot
  double constError_ = 0.2;                      /// constant for error term by qDot = c*qDot
  double solverBound_ = 10;                      /// the upper and lower bound for each status in solver
  double safetyDistance_ = 0.15;                 /// the safety distance between robot bar and obstacle
  double safetyIgnore_ = 0.2;                    /// the safety distance between robot bar and obstacle that directly ignore this contact
  std::vector<double> linkObstacleAvoid_;        /// the link that considers obstacle avoidance
  double robustJinvLambda_ = 0.001;              /// the lambda parameter for damped jacobian inverse
  double tolerance_ = 0.01;                      /// the tolerance for the end condition
  double maximumTime_ = 60;                      /// the maximum time for the planner (not used in this version)
  int jacobianSkipcount_ = 3;                    /// the skip count for the jacobian
  // double LowPassJoint_ = 0;                      /// the IIR low pass filter for the velocity
  LCQP_controller::FrankaModel frankaModel_;      /// the model of the robot

  LCQP_controller::Controller controller_;       /// the controller for the planner
  std::vector<Vector3d> obsList_;                 /// the list of obstacles 
  std::vector<double> obsRadiusList_;             /// the list of obstacles' radius

  ros::Publisher obstacle_pub_; /// the publisher for the obstacles in Rviz
  void publishObstacle(const std::vector<Vector3d>& obsList, const std::vector<double>& obsRadiusList);

  ros::Publisher distance_pub_; /// the publisher for the distance between robot and obstacles

  ros::Publisher traj_pub_;
  ros::Publisher vel_pub_;
  void publishTrajectory(const std::vector<Vector3d>& trajList, const Vector3d& velcmd);
};

}  // namespace franka_controllers