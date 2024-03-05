#include<LCQP_controller/LCQP_controller.hpp>

using namespace Eigen;

namespace LCQP_controller{
    Controller::Controller(){
    }

    Controller::~Controller(){
    }

    void Controller::initPlanner(const std::vector<double> goal,const LCQP_controller::FrankaModel frankaModel,const int nObsIn,const int nLinkObsIn, const std::vector<Eigen::Vector3d>& obsList,const std::vector<double>& obsRadiusList){
        goal_ = goal;
        frankaModel_ = frankaModel;

        // check if some parameters matches the requirement
        if (nObsIn != obsList.size()){
            throw std::invalid_argument("The number of obstacles does not match the size of the obstacle list");
        }
        if (nObsIn != obsRadiusList.size()){
            throw std::invalid_argument("The number of obstacles does not match the size of the obstacle radius list");
        }
        if (nLinkObsIn != linkObstacleAvoid_.size()){
            throw std::invalid_argument("The number of links that consider obstacle avoidance does not match the size of the link list");
        }
        if (jacobianSkipcount_ <= 0){
            throw std::invalid_argument("The skip count for the jacobian cannot be zero or negative");
        }
        if (jacobianSkipcount_ > 5){
            throw std::invalid_argument("The skip count for the jacobian is too large");
        }

        // define the dimension of the problem
        nLink = 7;
        nObs = nObsIn;
        nLinkObs = nLinkObsIn;
        nContacts = nObs * nLinkObs;
        if (nContacts <= nLink){
            nContacts = nLink;
        }
        nVariables = nLink + nContacts;
        xLast_.resize(nVariables);

        // initialize others things
        options.setPrintLevel(LCQPow::PrintLevel::NONE);
        options.setQPSolver(LCQPow::QPSolver::QPOASES_DENSE);
        options.setStationarityTolerance( 10e-2 );

        // initialize the matrices
        Q = MatrixXd::Identity(nVariables, nVariables);
        g = VectorXd::Zero(nVariables);

        L = MatrixXd::Zero(nContacts, nVariables);
        L.block(0, nLink, nContacts, nContacts) = MatrixXd::Identity(nContacts, nContacts);
        lbL = VectorXd::Zero(nContacts);
        ubL = VectorXd::Constant(nContacts, solverBound_);
        if (nContacts > nObs*nLinkObs){
            ubL.segment(nObs*nLinkObs, nContacts - nObs*nLinkObs) = VectorXd::Zero(nContacts - nObs*nLinkObs);
        }

        R = MatrixXd::Zero(nContacts, nVariables);
        lbR = VectorXd::Zero(nContacts);
        ubR = VectorXd::Constant(nContacts, solverBound_);

        A = MatrixXd::Zero(nContacts, nVariables);
        A.block(0, 0, nLink, nLink) = MatrixXd::Identity(nLink, nLink);

        lbA = VectorXd::Zero(nContacts);

        lb = VectorXd::Zero(nVariables);
        lb.segment(0, nLink) = VectorXd::Constant(nLink, -solverBound_);
        ub = VectorXd::Constant(nVariables, solverBound_);

        contactDistMtx = VectorXd::Zero(nContacts);
        vecSafety = VectorXd::Zero(nContacts);
        vecSafety.segment(0, nObs * nLinkObs) = VectorXd::Constant(nObs * nLinkObs, safetyDistance_);

        // initialize the the matrices for the first time to prevent empty initialization problem (same process of the nextStep)
        VectorXd q = frankaModel_.getJointState();
        Matrix4d T_EE = frankaModel_.getTransformation(q, 7);

        Vector3d posNow = T_EE.block(0, 3, 3, 1);
        Vector3d posGoal = Eigen::Map<Vector3d>(goal_.data(), goal_.size());
        Vector3d velToGoal = constMainTask_ * (posGoal - posNow) / (posGoal - posNow).norm();

        MatrixXd J = frankaModel_.getJacobian(q, 7);
        MatrixXd J_pos = J.block(0,0,3,7);
        MatrixXd invJ_pos = J_pos.transpose() * (J_pos * J_pos.transpose() + robustJinvLambda_ * MatrixXd::Identity(3,3)).completeOrthogonalDecomposition().pseudoInverse();

        double contactDist = 0;
        Vector3d contactNormal = Vector3d::Zero(3);
        MatrixXd contactTransJacobian = MatrixXd::Zero(3, nLink);


        for(int iObs = 0; iObs < nObs; iObs++){
            for (int iLink = 0; iLink < nLinkObs; iLink++) {
                int linkCodeNow = linkObstacleAvoid_[iLink];
                int iNum = iObs * nLinkObs + iLink;

                barSphereContact(q, linkCodeNow, obsList[iObs], obsRadiusList[iObs], contactDist, contactNormal, contactTransJacobian, 0);

                contactDistMtx(iNum) = contactDist;
                R.row(iNum).segment(0, nLink) = constContactTask_ * contactNormal.transpose() * contactTransJacobian;
                
                MatrixXd JDinv = contactTransJacobian.transpose() * (contactTransJacobian * contactTransJacobian.transpose() + robustJinvLambda_ * MatrixXd::Identity(3,3)).completeOrthogonalDecomposition().pseudoInverse();
                A.block(0, iNum + nLink, nLink, 1) = -JDinv * contactNormal;
            }
        }

        lbR = - contactDistMtx + vecSafety;
        lbA.segment(0, nLink) = invJ_pos * velToGoal;
    }

    // inline void copyMatrixToDouble(double *dst, const MatrixXd &src) {
    //     int nRows = src.rows();
    //     int nCols = src.cols();
    //     for (int iRow = 0; iRow < nRows; iRow++) {
    //         for (int iCol = 0; iCol < nCols; iCol++) {
    //             dst[iRow * nCols + iCol] = src(iRow, iCol);
    //         }
    //     }
    // }

    // inline void copyVectorToDouble(double *dst, const VectorXd &src) {
    //     int nRows = src.rows();
    //     for (int iRow = 0; iRow < nRows; iRow++) {
    //         dst[iRow] = src(iRow);
    //     }
    // }

    Eigen::VectorXd Controller::nextStep(const std::vector<Eigen::Vector3d>& obsList,const std::vector<double>& obsRadiusList){
        VectorXd q = frankaModel_.getJointState();
        Matrix4d T_EE = frankaModel_.getTransformation(q, 7);

        Vector3d posNow = T_EE.block(0, 3, 3, 1);
        Vector3d posGoal = Eigen::Map<Vector3d>(goal_.data(), goal_.size());
        Vector3d velToGoal = constMainTask_ * (posGoal - posNow) / (posGoal - posNow).norm();

        MatrixXd J = frankaModel_.getJacobian(q, 7);
        MatrixXd J_pos = J.block(0,0,3,7);
        MatrixXd invJ_pos = J_pos.transpose() * (J_pos * J_pos.transpose() + robustJinvLambda_ * MatrixXd::Identity(3,3)).completeOrthogonalDecomposition().pseudoInverse();

        //VectorXd contactDistMtx = VectorXd::Zero(nContacts);

        //MatrixXd Q = MatrixXd::Identity(nVariables, nVariables);
        //VectorXd g = VectorXd::Zero(nVariables);

        //MatrixXd L = MatrixXd::Zero(nContacts, nVariables);
        //L.block(0, nLink, nContacts, nContacts) = MatrixXd::Identity(nContacts, nContacts);
        //VectorXd lbL = VectorXd::Zero(nContacts);
        //VectorXd ubL = VectorXd::Constant(nContacts, solverBound_);
        //if (nContacts > nObs*nLinkObs){
        //    ubL.segment(nObs*nLinkObs, nContacts - nObs*nLinkObs) = VectorXd::Zero(nContacts - nObs*nLinkObs);
        //}

        //MatrixXd R = MatrixXd::Zero(nContacts, nVariables);
        //MatrixXd A = MatrixXd::Zero(nContacts, nVariables);
        //A.block(0, 0, nLink, nLink) = MatrixXd::Identity(nLink, nLink);

        double contactDist = 0;
        Vector3d contactNormal = Vector3d::Zero(3);
        MatrixXd contactTransJacobian = MatrixXd::Zero(3, nLink);

        //#pragma omp parallel for num_threads(2) collapse(2)
        {
            static int count_iter = 0;
            if (count_iter % jacobianSkipcount_ == 0)
            {
                for (int iObs = 0; iObs < nObs; iObs++)
                {
                    for (int iLink = 0; iLink < nLinkObs; iLink++)
                    {
                        int linkCodeNow = linkObstacleAvoid_[iLink];
                        int iNum = iObs * nLinkObs + iLink;

                        bool returnValid = barSphereContact(q, linkCodeNow, obsList[iObs], obsRadiusList[iObs], contactDist, contactNormal, contactTransJacobian, safetyIgnore_);

                        contactDistMtx(iNum) = contactDist;

                        if (returnValid)
                        {
                            R.row(iNum).segment(0, nLink) = constContactTask_ * contactNormal.transpose() * contactTransJacobian;
                            MatrixXd JDinv = contactTransJacobian.transpose() * (contactTransJacobian * contactTransJacobian.transpose() + robustJinvLambda_ * MatrixXd::Identity(3, 3)).completeOrthogonalDecomposition().pseudoInverse();
                            A.block(0, iNum + nLink, nLink, 1) = -JDinv * contactNormal;
                        }
                    }
                }
            }
            count_iter++;
        }

        lbR = - contactDistMtx + vecSafety;
        //lbR = - contactDistMtx + VectorXd::Constant(nContacts, safetyDistance_);
        // if (nContacts > nObs * nLinkObs) {
        //     lbR.segment(nObs * nLinkObs, nContacts - nObs * nLinkObs) = VectorXd::Zero(nContacts - nObs * nLinkObs);
        // }

        //VectorXd ubR = VectorXd::Constant(nContacts, solverBound_);

        //VectorXd lbA = VectorXd::Zero(nContacts);
        lbA.segment(0, nLink) = invJ_pos * velToGoal;

        //VectorXd lb = VectorXd::Zero(nVariables);
        //lb.segment(0, nLink) = VectorXd::Constant(nLink, -solverBound_);
        //VectorXd ub = VectorXd::Constant(nVariables, solverBound_);

        // copy value to double for solver
        // double Qp[nVariables * nVariables];
        // copyMatrixToDouble(Qp, Q);
        // double gp[nVariables];
        // copyVectorToDouble(gp, g);
        // double Lp[nContacts * nVariables];
        // copyMatrixToDouble(Lp, L);
        // double Rp[nContacts * nVariables];
        // copyMatrixToDouble(Rp, R);
        // double Ap[nContacts * nVariables];
        // copyMatrixToDouble(Ap, A);
        
        // double lbAp[nContacts];
        // copyVectorToDouble(lbAp, lbA);
        // // double ubAp[nContacts];
        // // copyVectorToDouble(ubAp, ubA);
        // double lbLp[nContacts];
        // copyVectorToDouble(lbLp, lbL);
        // double ubLp[nContacts];
        // copyVectorToDouble(ubLp, ubL);
        // double lbRp[nContacts];
        // copyVectorToDouble(lbRp, lbR);
        // double ubRp[nContacts];
        // copyVectorToDouble(ubRp, ubR);
        // double lbp[nVariables];
        // copyVectorToDouble(lbp, lb);
        // double ubp[nVariables];
        // copyVectorToDouble(ubp, ub);

        // set the planner
        LCQPow::LCQProblem lcqProblem_(nVariables,nContacts,nContacts);
        lcqProblem_.setOptions( options );

        LCQPow::ReturnValue retVal = lcqProblem_.loadLCQP(Q.data(), g.data(), 
            L.data(), R.data(), lbL.data(), ubL.data(), lbR.data(), ubR.data(), 
            A.data(), lbA.data(), lbA.data(), lb.data(), ub.data(), xLast_.data(), 0);
        
        int inError = 0;
        if (retVal != LCQPow::SUCCESSFUL_RETURN){
            inError = 1;
        }

        retVal = lcqProblem_.runSolver();
        if (retVal != LCQPow::SUCCESSFUL_RETURN){
            inError = 1;
        }

        double xOpt[nVariables];
        lcqProblem_.getPrimalSolution(xOpt);

        VectorXd qDot = constError_ * VectorXd::Map(xOpt, nLink);

        Vector3d difference = posGoal - posNow;
        if (difference.norm() >= tolerance_ && inError == 0) {
            for (int i = 0; i < nVariables; i++){
                xLast_[i] = xOpt[i];
            }
            return qDot;
        }else{
            VectorXd lastqDot = VectorXd::Map(xLast_.data(), nLink);
            return lastqDot;
        }
    }

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    bool Controller::barSphereContact(const VectorXd& q_vec, const int& to_ith_link, const Vector3d& obstaclePosition, const double& obstacleRadius, double &contactDistOut, Vector3d &contactNormalOut, MatrixXd &contactTransJacobianOut, const double& safetyIgnore){
        int frame = to_ith_link;
        int frame_last = to_ith_link - 1;

        Matrix4d T = frankaModel_.getTransformation(q_vec, frame);
        Matrix4d T_last = frankaModel_.getTransformation(q_vec, frame_last);

        Vector3d linkStart = T_last.block(0,3,3,1);
        Vector3d linkEnd = T.block(0,3,3,1);

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

        contactDistOut = contactDist;
        contactNormalOut = contactNormal;

        bool returnValid = contactDist < safetyIgnore;
        
        if (returnValid)
        {
            MatrixXd J = frankaModel_.getJacobian(q_vec, frame);
            MatrixXd contactTransJacobian = J.block(0,0,3,J.cols());
            contactTransJacobianOut = contactTransJacobian;
        }

        return returnValid;
        //return std::make_tuple(contactDist, contactPtObs, contactPtRobot, contactNormal, contactTransJacobian);
    }
}