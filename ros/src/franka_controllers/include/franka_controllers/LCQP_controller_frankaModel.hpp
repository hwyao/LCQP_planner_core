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
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

#include <LCQProblem.hpp>

using namespace Eigen;

namespace franka_controllers {

class LCQPControllerFrankaModel : public controller_interface::MultiInterfaceController<
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
  std::tuple<double, Vector3d, Vector3d, Vector3d, MatrixXd> barSphereContact(const VectorXd& q_vec, const int& to_ith_link, const Vector3d& obstaclePosition, const double& obstacleRadius) const;

  ros::Duration elapsed_time_;

  std::vector<double> goal_ = {0.1, 0.450, 0.5}; /// Goal of the planner. Here we accept the goal as position.
  double constMainTask_ = 0.1;                   /// constant for main task c*J*dx
  double constContactTask_ = 0.006;              /// constant for collision avoidance h*J1_1*N1_1*qdot
  double constError_ = 0.2;                      /// constant for error term by qDot = c*qDot
  double solverBound_ = 10;                      /// the upper and lower bound for each status in solver
  double safetyDistance_ = 0.15;                 /// the safety distance between robot bar and obstacle
  std::vector<double> linkObstacleAvoid_;        /// the link that considers obstacle avoidance
  double robustJinvLambda_ = 0.001;              /// the lambda parameter for damped jacobian inverse
  double tolerance_ = 0.01;                      /// the tolerance for the end condition
  double maximumTime_ = 60;                      /// the maximum time for the planner (not used in this version)
  double LowPassJoint_ = 0;                      /// the IIR low pass filter for the velocity
  double LowPassObs_ = 0;                        /// the IIR low pass filter for the obstacle position

  double obstacleRadius_; /// the radius of the obstacle

  int nLink;      /// the number of links
  int nObs;       /// the number of obstacles
  int nLinkObs;   /// the number of links that consider obstacle avoidance
  int nContacts;  /// the number of contacts
  int nVariables; /// the number of variables in solver

  //LCQPow::LCQProblem lcqProblem_; /// the LCQP problem solver
  std::vector<double> xLast_;     /// the last solution of the solver

  // obstacle subscriber
  Vector3d obstaclePosition_; /// the position of the obstacle

  std::mutex obstacle_pose_mutex_;
  Vector3d obstaclePosition_read_; /// the position of the obstacle
  
  ros::Subscriber sub_obstacle_pose_;
  void obstaclePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_controllers