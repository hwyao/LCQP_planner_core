% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Riddhiman Laha
classdef ControllerFeasibility < IController
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
        function controller = ControllerFeasibility(robotModel,obstacleList,goal)
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
%             % read the status
%             for iObs = 1:numel(controller.obstacleList)
%                 controller.obstacleList{iObs}.updateStatus(obsPosList(iObs,:))
%             end

            % some constants (could be later moved into the class property)
            cMainTask = 1;
            hContact = 0.001;
            changeMax = 100;
            saftyDist = 0.1;
            linkCode = [3,4,5,7];
            lambda = 0.001;

            % setup path solver
            % Set the bounds of unknowns for PATH Solver
            for j = 1 : 14
                l(j) = -Inf; u(j) = Inf;
            end
            l(1, 8:14) = 0;   % since complementarity velocity is always >= 0 
            z = zeros(14, 1); % first 7 -> q and last 7 -> compensating velocities
            
            % get coordiate status
            qNow = controller.q;
            fkNow = controller.robotModel.fkm(qNow);
            posNow = fkNow.translation.vec3;
            posDQ = fkNow.translation;
            rotNow = fkNow.rotation;

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
            
            contactDistMtx = zeros(nContacts,1);
            contactNormalMtx = zeros(nContacts,3);
            contactJacobMtx = zeros(4,nLink,nContacts); 

            for iObs = 1:nObs
                for iLink = 1:numel(linkCode)
                    linkCodeNow = linkCode(iLink);
                    [contactDist, ~, ~, ...
                     contactNormal, contactTransJacobian] = controller.robotModel.detectContact(controller.obstacleList{iObs},qNow,linkCodeNow);
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

            [z, f, J] = pathmcp(z, l, u, 'mcpfuncjacEval');
            q_o = z(1:nLink);
            controller.q = q_o;
            
            % update and send to vrep
        end
    end
end