% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef ControllerLCQP < IController
    %CONTROLLERLCQP The planner based on Quadratic Programs with Linear 
    % Complementarity Constraints 
    
    properties
        %%%%%%%%%%%%%% handle properties %%%%%%%%%%%%%%
        robotModel = RobotModelFrankaBar.empty

        obstacleList = {}

        %%%%%%%%%%%%%% simulation properties %%%%%%%%%%%%%%
        dt = 1e-2

        toleranceEnd = 1e-2

        maxEndCount = 3

        maxStep = 1000

        %%%%%%%%%%%%% simulation status %%%%%%%%%%%%%%%%%%%%%
        q

        currentEndCount = 0

        currentStep = 0

        xLast

        %%%%%%%%%%%%% simulation goal %%%%%%%%%%%%%%%%%%%%%%%
        goal(3,1) double
    end
    
    methods
        function controller = ControllerLCQP(robotModel,obstacleList,goal)
            controller.robotModel = robotModel;
            controller.obstacleList = obstacleList;
            controller.q = robotModel.fetchStatus();
            controller.goal = goal;
        end
    end

    methods
        function isReached = goalReached(controller)
            xCurrent = controller.robotModel.fkm(controller.q).translation.vec3;
            if norm(xCurrent-controller.goal)<=controller.toleranceEnd
                isReached = true;
            else
                isReached = false;
            end
        end

        function nextStep(controller,obsPosList)
            % update the obstacle positions
            for iObs = 1:numel(controller.obstacleList)
                controller.obstacleList{iObs}.updateStatus(obsPosList(iObs,:));
            end

            % some constants (could be later moved into the class property)
            cMainTask = 1;
            hContact = 0.001;
            changeMax = 100;
            saftyDist = 0.1;
            linkCode = [3,4,5,7];
            lambda = 0.001;

            % get coordiate status
            qNow = controller.q;
            fkNow = controller.robotModel.fkm(qNow);
            posNow = fkNow.translation.vec3;
            posGoal = controller.goal;
            velToGoal = cMainTask * (posGoal - posNow) / norm(posGoal - posNow);
            velToGoalDQ = DQ(velToGoal);

            J = controller.robotModel.poseJacobian(qNow);
            J_pos = controller.robotModel.kinematic.translation_jacobian(J,fkNow);
            %invJ = pinv(J);
            invJ_pos = J_pos' * pinv(J_pos*J_pos' + lambda * eye(4));

            nLink = controller.robotModel.kinematic.n_links;
            nObs = numel(controller.obstacleList);
            nLinkObs = numel(linkCode);
            nContacts = nObs*nLinkObs;
            if (nContacts <= nLink)
                nContacts = nLink;
            end
            nVariables = nLink + nContacts;

            % Algorithm explained:
            % The x for LCQP is = [qdot_1,qdot_2,qdot_3,....,qdot_nLink, v1_1, v1_2, ..., vnObs_nLinkObs];
            % For which:
            % qdot: the next step of q (our target)
            % vm_n: the activacted velocity between obstacle m and link n
            % 
            % The Lx = [v1_1, v1_2, ..., vnObs_nLinkObs] is complementary to
            % Rx = [h*J1_1*N1_1*qdot,h*J1_2*N1_2*qdot,...,h*JnObs_nLinkObs*NnObs_nLinkObs*qdot];
            % The lower bound of Rx contains safty distance, which will
            % activate corresponding v when the lower bound attempt to turn positive. 
            contactDistMtx = zeros(nContacts,1);
            contactNormalMtx = zeros(nContacts,3);
            contactJacobMtx = zeros(4,nLink,nContacts);

            Q = diag([repelem(1,nVariables)]);
            g = zeros(nVariables,1);
            
            L = [zeros(nContacts,nLink),diag(repelem(1,nContacts))];
            lbL = zeros(nContacts, 1);
            ubL = changeMax * ones(nContacts, 1);
            if (nContacts > nObs*nLinkObs)
                ubL(nObs*nLinkObs+1:end) = 0;
            end
            
            R = zeros(nContacts,nVariables);
            A = [[diag(repelem(1,nLink));zeros(nContacts-nLink,nLink)],[zeros(nContacts,nContacts)]];

            % % % % my additions
            % % % mylinkCode = [1, 2, 3, 4, 5, 6, 7];
            % % % for iObs = 1:nObs
            % % %     for iLink = 1:numel(mylinkCode)
            % % %         linkCodeNow = mylinkCode(iLink);
            % % %         [contactDist, ~, ~, ...
            % % %         contactNormal, contactTransJacobian, ...
            % % %         contactTransJacobianGeometric] = controller.robotModel.detectContact(controller.obstacleList{iObs},qNow,linkCodeNow);
            % % %         iNum = (iObs-1)*nLinkObs + iLink;
            % % %         contactDistMtx(iNum) = contactDist;
            % % %         contactNormalMtx(iNum,:) = contactNormal;
            % % %         contactJacobMtx(:,1:linkCodeNow,iNum) = contactTransJacobian;
            % % %         contactNormalDQ = DQ(contactNormal);
            % % % 
            % % %         R(iNum,1:nLink) = hContact * contactNormalDQ.vec4' * contactJacobMtx(:,:,iNum); % validate this later.
            % % %         J_temp = contactJacobMtx(:,:,iNum);
            % % %         JDinv = J_temp' * pinv(J_temp*J_temp' + lambda * eye(4));
            % % %         A(1:nLink,iNum+nLink) = -JDinv * contactNormalDQ.vec4;
            % % %     end
            % % % end
            % % % % end of my additions

            for iObs = 1:nObs
                for iLink = 1:numel(linkCode)
                    linkCodeNow = linkCode(iLink);
                    [contactDist, ~, ~, ...
                    contactNormal, contactTransJacobian, ...
                    contactTransJacobianGeometric] = controller.robotModel.detectContact(controller.obstacleList{iObs},qNow,linkCodeNow);
                    iNum = (iObs-1)*nLinkObs + iLink;
                    contactDistMtx(iNum) = contactDist;
                    contactNormalMtx(iNum,:) = contactNormal;
                    contactJacobMtx(:,1:linkCodeNow,iNum) = contactTransJacobian;
                    contactNormalDQ = DQ(contactNormal);
    
                    R(iNum,1:nLink) = hContact * contactNormalDQ.vec4' * contactJacobMtx(:,:,iNum); % validate this later.
                    J_temp = contactJacobMtx(:,:,iNum);
                    JDinv = J_temp' * pinv(J_temp*J_temp' + lambda * eye(4));
                    A(1:nLink,iNum+nLink) = -JDinv * contactNormalDQ.vec4;
                end
            end

            lbR = - contactDistMtx + saftyDist;
            if (nContacts > nObs*nLinkObs)
                lbR(nObs*nLinkObs+1:end) = 0;
            end
            ubR = changeMax * ones(nContacts, 1);

            lbA = zeros(nContacts,1);
            ubA = zeros(nContacts,1);
            lbA(1:nLink) = invJ_pos * velToGoalDQ.vec4;
            ubA(1:nLink) = invJ_pos * velToGoalDQ.vec4;

            lb = [-changeMax*ones(nLink,1);zeros(nContacts,1)];
            ub = changeMax*ones(nVariables,1);

            params.complementarityTolerance = 1e-7;
            if controller.currentStep == 0
                params.x0 = zeros(nVariables,1);
            else
                params.x0 = controller.xLast;
            end

            [x, ~, ~] = LCQPow(Q, g, L, R, lbL, ubL, lbR, ubR, A, lbA, ubA, lb, ub, params);
            controller.xLast = x;
            qdot = x(1:nLink,1);
            controller.q = controller.q + hContact * qdot; 

            % update the status
            controller.sendAndStep()
        end
    end
end
