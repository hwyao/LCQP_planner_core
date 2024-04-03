/**
 * @file FrankaModel.hpp
 * @author Haowen Yao
 * @brief This is a dirty implementation of the FrankaModel that pass kinematic infomation to controller from different sources
 *        The usecase of it is the LCQP_controller_multipleObstacle.cpp, which uses LCQP_controller and FrankaModel to work with 
 *        multiple obstacles in different environment from different data sources. 
 *        The interface needed by the controllers are: getJointState(), getTransformation(), getJacobian(), which should strictly
 *        follow the function signature and return type. The rest (like how to initialize) can be modified to fit the usecase.
 * @date 2024-03-02
 * 
 * @copyright Copyright (c) 2024
 */
#ifndef FRANKA_MODEL_HPP
#define FRANKA_MODEL_HPP

#include <Eigen/Dense>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

namespace LCQP_controller {
    class FrankaModel {
    public:
        FrankaModel();
        FrankaModel(franka_hw::FrankaStateHandle state_handle, franka_hw::FrankaModelHandle model_handle);

        ~FrankaModel();

        Eigen::VectorXd getJointState() const;

        Eigen::Matrix4d getTransformation(Eigen::VectorXd q, int link) const;

        Eigen::MatrixXd getJacobian(Eigen::VectorXd q, int link) const;

    private:
        std::shared_ptr<franka_hw::FrankaStateHandle> state_handle_;
        
        std::shared_ptr<franka_hw::FrankaModelHandle> model_handle_;
};


} // namespace LCQP_controller

#endif // FRANKA_MODEL_HPP