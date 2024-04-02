#include <franka_controllers/LCQP_controller_frankaModel.hpp>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#define DEBUG_PRINT
#undef DEBUG_PRINT

using namespace Eigen;

namespace franka_controllers {

bool LCQPControllerFrankaModel::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  ROS_INFO("LCQPControllerFrankaModel: LCQP controller init");

  // subscribe to obstacle pose
  sub_obstacle_pose_ = node_handle.subscribe("obstacle_pose", 1, &LCQPControllerFrankaModel::obstaclePoseCallback, this);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("LCQPControllerFrankaModel: Could not read parameter arm_id");
    return false;
  }
                                            
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse joint names");
    return false;
  }

  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("LCQPControllerFrankaModel: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "LCQPControllerFrankaModel: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "LCQPControllerFrankaModel: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "LCQPControllerFrankaModel: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "LCQPControllerFrankaModel: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "LCQPControllerFrankaModel: Error getting position joint interface from hardware!");
    return false;
  }

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "LCQPControllerFrankaModel: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(velocity_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "LCQPControllerFrankaModel: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  // disable the lowpass filter of franka (when testing controller for the first time)
  

  // load parameters
  if(!node_handle.getParam("goal", goal_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse goal");
  }
  if(!node_handle.getParam("constMainTask", constMainTask_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse constMainTask");
  }
  if(!node_handle.getParam("constContactTask", constContactTask_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse constContactTask");
  }
  if(!node_handle.getParam("constError", constError_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse constError");
  }
  if(!node_handle.getParam("solverBound", solverBound_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse solverBound");
  }
  if(!node_handle.getParam("safetyDistance", safetyDistance_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse safetyDistance");
  }
  if(!node_handle.getParam("linkObstacleAvoid", linkObstacleAvoid_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse linkObstacleAvoid");
  }
  if(!node_handle.getParam("robustJinvLambda", robustJinvLambda_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse robustJinvLambda");
  }
  if(!node_handle.getParam("tolerance", tolerance_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse tolerance");
  }
  if(!node_handle.getParam("maximumTime", maximumTime_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse maximumTime");
  }
  
  std::vector<double> obstaclePosition_Param;
  if(!node_handle.getParam("obstaclePosition", obstaclePosition_Param)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse obstaclePosition");
  }
  obstaclePosition_ << obstaclePosition_Param[0], obstaclePosition_Param[1], obstaclePosition_Param[2];
  obstaclePosition_read_ << obstaclePosition_Param[0], obstaclePosition_Param[1], obstaclePosition_Param[2];

  double obstacleRadius;
  if(!node_handle.getParam("obstacleRadius", obstacleRadius_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse obstacleRadius");
  }
  if(!node_handle.getParam("LowPassJoint", LowPassJoint_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse LowPass");
  }
  if(!node_handle.getParam("LowPassObs", LowPassObs_)){
    ROS_ERROR("LCQPControllerFrankaModel: Could not parse LowPassObs");
  }

  // set the running variables
  nLink = 7;
  nObs = 1;
  nLinkObs = linkObstacleAvoid_.size();
  nContacts = nObs * nLinkObs;
  if (nContacts <= nLink){
    nContacts = nLink;
  }
  nVariables = nLink + nContacts;

  xLast_.resize(nVariables);
  for (int i = 0; i < nVariables; i++){
      xLast_[i] = 0;
  }
  //lcqProblem_.switchToSparseMode();

  ROS_INFO("LCQPControllerFrankaModel: LCQP controller initialized");

  return true;
}

void LCQPControllerFrankaModel::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

inline void copyMatrixToDouble(double *dst, const MatrixXd &src) {
    int nRows = src.rows();
    int nCols = src.cols();
    for (int iRow = 0; iRow < nRows; iRow++) {
        for (int iCol = 0; iCol < nCols; iCol++) {
            dst[iRow * nCols + iCol] = src(iRow, iCol);
        }
    }
}

inline void copyVectorToDouble(double *dst, const VectorXd &src) {
    int nRows = src.rows();
    for (int iRow = 0; iRow < nRows; iRow++) {
        dst[iRow] = src(iRow);
    }
}

void LCQPControllerFrankaModel::update(const ros::Time& /*time*/, const ros::Duration& period) {
  //ROS_INFO("LCQPControllerFrankaModel: Current time: start");
  elapsed_time_ += period;

  franka::RobotState robot_state = state_handle_->getRobotState();
  
  VectorXd qNow = VectorXd::Map(robot_state.q.data(), nLink);
  Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  
  Vector3d posNow(transform.translation());
  Vector3d posGoal = Eigen::Map<Vector3d>(goal_.data(), goal_.size());
  Vector3d velToGoal = constMainTask_ * (posGoal - posNow) / (posGoal - posNow).norm();

  std::array<double, 42> J_array = model_handle_->getZeroJacobian(franka::Frame::kFlange);
  MatrixXd J = MatrixXd::Map(J_array.data(), 6, 7);

  MatrixXd J_pos = J.block(0,0,3,7);
  //MatrixXd J_pos = franka_kinematics_.translation_jacobian(J,fkNow);

  MatrixXd invJ_pos = J_pos.transpose() * (J_pos * J_pos.transpose() + robustJinvLambda_ * MatrixXd::Identity(3,3)).completeOrthogonalDecomposition().pseudoInverse();
  //MatrixXd invJ_pos = J_pos.transpose();

  VectorXd contactDistMtx = VectorXd::Zero(nContacts);
  // MatrixXd contactNormalMtx = MatrixXd::Zero(nContacts,3);
  // std::vector<MatrixXd> contactJacobMtx(nContacts, MatrixXd::Zero(3,nLink));

  MatrixXd Q = MatrixXd::Identity(nVariables, nVariables);
  VectorXd g = VectorXd::Zero(nVariables);

  MatrixXd L = MatrixXd::Zero(nContacts, nVariables);
  L.block(0, nLink, nContacts, nContacts) = MatrixXd::Identity(nContacts, nContacts);
  VectorXd lbL = VectorXd::Zero(nContacts);
  VectorXd ubL = VectorXd::Constant(nContacts, solverBound_);
  if (nContacts > nObs*nLinkObs){
      ubL.segment(nObs*nLinkObs, nContacts - nObs*nLinkObs) = VectorXd::Zero(nContacts - nObs*nLinkObs);
  }

  MatrixXd R = MatrixXd::Zero(nContacts, nVariables);
  MatrixXd A = MatrixXd::Zero(nContacts, nVariables);
  A.block(0, 0, nLink, nLink) = MatrixXd::Identity(nLink, nLink);

  // // print the obstacle position
  //ROS_INFO_STREAM_THROTTLE(0.2, "Main"<<obstaclePosition_[0] << " " << obstaclePosition_[1] << " " << obstaclePosition_[2]);
  for (int iLink = 0; iLink < nLinkObs; iLink++) {
      int linkCodeNow = linkObstacleAvoid_[iLink];
      
      std::tuple<double, Vector3d, Vector3d, Vector3d, MatrixXd> contactInfo = 
          barSphereContact(qNow, linkCodeNow, obstaclePosition_, obstacleRadius_);
      double contactDist = std::get<0>(contactInfo);
      Vector3d contactNormal = std::get<3>(contactInfo);
      MatrixXd contactTransJacobian = std::get<4>(contactInfo);

      // print the contact info
      // std::cout << "contactDist: \n" << contactDist <<std::endl;
      // std::cout << "contactNormal: \n" << contactNormal <<std::endl;
      // std::cout << "contactTransJacobian: \n" << contactTransJacobian <<std::endl;

      contactDistMtx(iLink) = contactDist;
      // contactNormalMtx.row(iNum) = contactNormal;
      // contactJacobMtx[iNum].block(0, 0, 3, nLink) = contactTransJacobian;

      R.row(iLink).segment(0, nLink) = constContactTask_ * contactNormal.transpose() * contactTransJacobian;
      
      //MatrixXd JDinv = contactTransJacobian.transpose();
      MatrixXd JDinv = contactTransJacobian.transpose() * (contactTransJacobian * contactTransJacobian.transpose() + robustJinvLambda_ * MatrixXd::Identity(3,3)).completeOrthogonalDecomposition().pseudoInverse();
      
      A.block(0, iLink + nLink, nLink, 1) = - constContactTask_ * JDinv * contactNormal;
  }

  VectorXd lbR = -contactDistMtx + VectorXd::Constant(nContacts, safetyDistance_);

  if (nContacts > nObs * nLinkObs) {
      lbR.segment(nObs * nLinkObs, nContacts - nObs * nLinkObs) = VectorXd::Zero(nContacts - nObs * nLinkObs);
  }
  VectorXd ubR = VectorXd::Constant(nContacts, solverBound_);

  VectorXd lbA = VectorXd::Zero(nContacts);
  // VectorXd ubA = VectorXd::Zero(nContacts); // reuse the same vector
  lbA.segment(0, nLink) = invJ_pos * velToGoal;
  // ubA.segment(0, nLink) = invJ_pos * velToGoal; // reuse the same vector

  VectorXd lb = VectorXd::Zero(nVariables);
  lb.segment(0, nLink) = VectorXd::Constant(nLink, -solverBound_);
  VectorXd ub = VectorXd::Constant(nVariables, solverBound_);

  // copy value to double for solver
  double Qp[nVariables * nVariables];
  copyMatrixToDouble(Qp, Q);
  double gp[nVariables];
  copyVectorToDouble(gp, g);
  double Lp[nContacts * nVariables];
  copyMatrixToDouble(Lp, L);
  double Rp[nContacts * nVariables];
  copyMatrixToDouble(Rp, R);
  double Ap[nContacts * nVariables];
  copyMatrixToDouble(Ap, A);
  
  double lbAp[nContacts];
  copyVectorToDouble(lbAp, lbA);
  // double ubAp[nContacts];
  // copyVectorToDouble(ubAp, ubA);
  double lbLp[nContacts];
  copyVectorToDouble(lbLp, lbL);
  double ubLp[nContacts];
  copyVectorToDouble(ubLp, ubL);
  double lbRp[nContacts];
  copyVectorToDouble(lbRp, lbR);
  double ubRp[nContacts];
  copyVectorToDouble(ubRp, ubR);
  double lbp[nVariables];
  copyVectorToDouble(lbp, lb);
  double ubp[nVariables];
  copyVectorToDouble(ubp, ub);

  // set the planner
  LCQPow::LCQProblem lcqProblem_(nVariables,nContacts,nContacts);
  LCQPow::Options options;
  options.setPrintLevel(LCQPow::PrintLevel::NONE);
  options.setQPSolver(LCQPow::QPSolver::QPOASES_DENSE);
  options.setStationarityTolerance( 10e-3 );
  lcqProblem_.setOptions( options );

  LCQPow::ReturnValue retVal = lcqProblem_.loadLCQP(Qp, gp, Lp, Rp, lbLp, ubLp, lbRp, ubRp, Ap, lbAp, lbAp, lbp, ubp, xLast_.data(), 0);
  int inError = 0;
  if (retVal != LCQPow::SUCCESSFUL_RETURN){
      ROS_ERROR("LCQPControllerFrankaModel: LCQP solver failed loading");
      inError = 1;
  }

  retVal = lcqProblem_.runSolver();
  if (retVal != LCQPow::SUCCESSFUL_RETURN){
      ROS_ERROR("LCQPControllerFrankaModel: LCQP solver failed solving");
      inError = 1;
  }

  double xOpt[nVariables];
  lcqProblem_.getPrimalSolution(xOpt);

  VectorXd qNext = VectorXd::Zero(nLink);

  // first nLink variables of xOpt are joint angles
  VectorXd qDot = constError_ * VectorXd::Map(xOpt, nLink);
  //std::cout << "qDot: \n" << qDot.transpose() <<std::endl;

  // filter the qDot (is disabled by setting LowPass_ = 0. But anyway we keep this here.)
  static VectorXd qDotFiltered = qDot;
  double alpha = LowPassJoint_ * 0.001;
  qDotFiltered = qDot + alpha * (qDotFiltered - qDot);

  // save the result as initial guess for next iteration
  for (int i = 0; i < nVariables; i++){
      xLast_[i] = xOpt[i];
  }

  // Set the joint velocity command
  Vector3d difference = posGoal - posNow;

  if (difference.norm() >= tolerance_ && inError == 0) {
    for (size_t i = 0; i < 7; ++i) {
      velocity_joint_handles_[i].setCommand(qDotFiltered(i));
    }
  }else{
      if(inError == 1){
          ROS_INFO_ONCE("LCQPControllerFrankaModel: LCQP solver failed");
      }else{
          ROS_INFO_ONCE("LCQPControllerFrankaModel: Goal reached with tolerance %f", tolerance_);
      }
      
      for (size_t i = 0; i < 7; ++i) {
        velocity_joint_handles_[i].setCommand(0);
      }
  }

  //ROS_INFO("LCQPControllerFrankaModel: Current time: finish");
  std::lock_guard<std::mutex> guard(obstacle_pose_mutex_);
  obstaclePosition_ = obstaclePosition_read_;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

std::tuple<double, Vector3d, Vector3d, Vector3d, MatrixXd> LCQPControllerFrankaModel::barSphereContact(const VectorXd& q_vec, const int& to_ith_link, const Vector3d& obstaclePosition, const double& obstacleRadius) const{
    franka::Frame frame = static_cast<franka::Frame>(to_ith_link);
    franka::Frame frame_last = static_cast<franka::Frame>(to_ith_link-1);

    std::array<double, 16> T_array = model_handle_->getPose(frame);
    Matrix4d T = Matrix4d::Map(T_array.data(), 4, 4);

    std::array<double, 16> T_last_array = model_handle_->getPose(frame_last);
    Matrix4d T_last = Matrix4d::Map(T_last_array.data(), 4, 4);

    Vector3d linkStart(T_last(0,3), T_last(1,3), T_last(2,3));
    Vector3d linkEnd(T(0,3), T(1,3), T(2,3));

    double linkLength = (linkStart - linkEnd).norm();
    Vector3d obstacleCenter = obstaclePosition;
    
    Vector3d startEndVecNorm = (linkEnd - linkStart) / linkLength;
    Vector3d startObstacleVector = obstacleCenter - linkStart;
    double projectToLinkLength = startEndVecNorm.dot(startObstacleVector);
    double projectDirection = sgn(projectToLinkLength);

    Vector3d contactPtRobot;
    if (projectDirection == -1)
    {
        contactPtRobot = linkStart;
    }
    else if (projectToLinkLength > linkLength)
    {
        contactPtRobot = linkEnd;
    }
    else
    {
        contactPtRobot = linkStart + projectToLinkLength * startEndVecNorm;
    }

    Vector3d contactNormal;
    Vector3d contactPtObs;
    if (contactPtRobot == obstacleCenter)
    {
        contactNormal = Vector3d(0, 0, 0);
        contactPtObs = obstacleCenter;
    }
    else
    {
        contactNormal = (contactPtRobot - obstacleCenter) / (contactPtRobot - obstacleCenter).norm();
        contactPtObs = obstacleCenter + obstacleRadius * contactNormal;
    }

    double contactDist = (contactPtRobot - contactPtObs).norm();

    MatrixXd J = MatrixXd::Map(model_handle_->getZeroJacobian(frame).data(), 6, 7);
    MatrixXd contactTransJacobian = J.block(0,0,3,J.cols());

    return std::make_tuple(contactDist, contactPtObs, contactPtRobot, contactNormal, contactTransJacobian);
}

void LCQPControllerFrankaModel::obstaclePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> guard(obstacle_pose_mutex_);
  double readX = msg->pose.position.x;
  double readY = msg->pose.position.y;
  double readZ = msg->pose.position.z;
  double alpha = LowPassObs_ * 0.001;
  obstaclePosition_read_[0] = readX + alpha*(obstaclePosition_read_[0] - readX);
  obstaclePosition_read_[1] = readY + alpha*(obstaclePosition_read_[1] - readY);
  obstaclePosition_read_[2] = readZ + alpha*(obstaclePosition_read_[2] - readZ);
  //ROS_INFO_STREAM_THROTTLE(0.2, "Read"<<obstaclePosition_[0] << " " << obstaclePosition_[1] << " " << obstaclePosition_[2]);
}


}  // namespace franka_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::LCQPControllerFrankaModel,
                       controller_interface::ControllerBase)