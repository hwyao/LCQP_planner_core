#include<LCQP_controller/FrankaModel.hpp>

namespace LCQP_controller {
    // the do nothing version of the interface
    FrankaModel::FrankaModel() {
    }

    FrankaModel::~FrankaModel() {
    }

    // Eigen::VectorXd FrankaModel::getJointState() const {
    //     Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
    //     return q;
    // }

    // Eigen::Matrix4d FrankaModel::getTransformation(Eigen::VectorXd q, int link) {
    //     Eigen::Matrix4d T;
    //     T.setIdentity();
    //     return T;
    // }

    // Eigen::MatrixXd FrankaModel::getJacobian(Eigen::VectorXd q, int link) {
    //     Eigen::MatrixXd J;
    //     J.setZero(6, 7);
    //     return J;
    // }

    // the franka_ros version of the interface
    FrankaModel::FrankaModel(franka_hw::FrankaStateHandle state_handle, franka_hw::FrankaModelHandle model_handle){
        state_handle_ = std::make_shared<franka_hw::FrankaStateHandle>(state_handle);
        model_handle_ = std::make_shared<franka_hw::FrankaModelHandle>(model_handle);
    }

    Eigen::VectorXd FrankaModel::getJointState() const {
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::VectorXd q = Eigen::VectorXd::Map(robot_state.q.data(), 7);
        return q;
    }

    Eigen::Matrix4d FrankaModel::getTransformation(Eigen::VectorXd q, int link) const {
        franka::Frame frame = static_cast<franka::Frame>(link);
        return Eigen::Matrix4d::Map(model_handle_->getPose(frame).data(), 4, 4);;
    }
    
    Eigen::MatrixXd FrankaModel::getJacobian(Eigen::VectorXd q, int link) const{
        franka::Frame frame = static_cast<franka::Frame>(link);
        return Eigen::MatrixXd::Map(model_handle_->getZeroJacobian(frame).data(), 6, 7);
    }

} // namespace LCQP_controller