#ifndef LCQP_CONTROLLER_HPP
#define LCQP_CONTROLLER_HPP

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <LCQP_controller/FrankaModel.hpp>
#include <LCQProblem.hpp>

using namespace Eigen;

namespace LCQP_controller {
    class Controller
    {
    public:
        Controller();
        ~Controller();
        void initPlanner(const std::vector<double> goal,const LCQP_controller::FrankaModel frankaModel,const int nObsIn,const int nLinkObsIn, const std::vector<Eigen::Vector3d>& obsList,const std::vector<double>& obsRadiusList);
        VectorXd nextStep(const std::vector<Vector3d>& obsList,const std::vector<double>& obsRadiusList);
        bool barSphereContact(const VectorXd& q_vec, const int& to_ith_link, const Vector3d& obstaclePosition, const double& obstacleRadius, double &contactDistOut, Vector3d &contactNormalOut, MatrixXd &contactTransJacobianOut, const double& safetyIgnore);

        std::vector<double> goal_ = {0.1, 0.450, 0.5};         /// Goal of the planner. Here we accept the goal as position.
        double constMainTask_ = 0.1;                           /// constant for main task c*J*dx
        double constContactTask_ = 0.006;                      /// constant for collision avoidance h*J1_1*N1_1*qdot
        double constError_ = 0.2;                              /// constant for error term by qDot = c*qDot
        double solverBound_ = 100;                             /// the upper and lower bound for each status in solver
        double safetyDistance_ = 0.15;                         /// the safety distance between robot bar and obstacle
        double safetyIgnore_ = 0.2;                            /// the safety distance between robot bar and obstacle that directly ignore this contact
        std::vector<double> linkObstacleAvoid_ = {2, 3, 4, 6}; /// the link that considers obstacle avoidance
        double robustJinvLambda_ = 0.001;                      /// the lambda parameter for damped jacobian inverse
        double tolerance_ = 0.01;                              /// the tolerance for the end condition
        int jacobianSkipcount_ = 1;                            /// the skip count for the jacobian

        std::vector<double> xLast_;     /// the last solution of the solver
        LCQP_controller::FrankaModel frankaModel_;

        // buffered variables to increase the performance
        Matrix<double, -1, -1, RowMajor> Q;
        VectorXd g;
        Matrix<double, -1, -1, RowMajor> L;
        VectorXd lbL;
        VectorXd ubL;
        Matrix<double, -1, -1, RowMajor> R;
        VectorXd lbR;
        VectorXd ubR;
        Matrix<double, -1, -1, RowMajor> A;
        VectorXd lbA;                              // lbA and ubA are the same
        VectorXd lb;
        VectorXd ub;

        VectorXd contactDistMtx;
        VectorXd vecSafety;

        LCQPow::Options options;

    private:
        int nLink;      /// the number of links
        int nObs;       /// the number of obstacles
        int nLinkObs;   /// the number of links that consider obstacle avoidance
        int nContacts;  /// the number of contacts
        int nVariables; /// the number of variables in solver
    };
} // namespace LCQP_controller

#endif // LCQP_CONTROLLER_HPP

