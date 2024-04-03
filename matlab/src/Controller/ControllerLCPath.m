%CONTROLLERLCPATH The planner based on Path Solver with Linear Complementarity 
% Constraints

% Contributor: Anirban Sinha, Riddhiman Laha
classdef ControllerLCPath < IController   
    % inherited propeties
    properties
        robotModel = RobotModelFrankaBar.empty
        obstacleList = {}
        
        dt = 1e-2
        toleranceEnd = 1e-2
        maxEndCount = 3
        maxStep = 1000
        
        q
        tick
        currentEndCount = 0
        currentStep = 0
    end

    % planner specific properties
    properties
        % (Last) output of the planner. Stored for better initial guess of later solution. 
        z(:,1) double
        
        % Goal of the planner. Here we accept the goal as position.
        goal(3,1) double

        % constant for main task c*J*dx
        constMainTask = 0.5

        % constant for collision avoidance
        constContactTask = 0.01

        % the link that considers obstacle avoidance
        linkObstacleAvoid = [3,4,5,7]
    end
    
    methods
        function controller = ControllerLCPath(robotModel,obstacleList,goal)
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
            
            % update obstacle positions
            for iObs = 1:numel(controller.obstacleList)
                controller.obstacleList{iObs}.updateStatus(obsPosList(iObs,:))
            end

            % get constants 
            cMainTask = controller.constMainTask;
            beta = controller.constContactTask;
            linkCode = controller.linkObstacleAvoid;

            ht = tic;
            % get coordiate status
            qNow = controller.q;
            fkNow = controller.robotModel.fkm(qNow);
            posNow = fkNow.translation.vec3;
            posGoal = controller.goal;
            velToGoal = cMainTask * (posGoal - posNow) / norm(posGoal - posNow);

            % prepare size information
            nLink = controller.robotModel.kinematic.n_links;
            nObs = numel(controller.obstacleList);

            % prepare data for LCPath
            contactDistMtx = zeros(1, numel(linkCode));            
            contactNormalMtx = zeros(6, numel(linkCode));
            contactJacobMtx = zeros(6,nLink,numel(linkCode));

            for iLink = 1:numel(linkCode)
                linkCodeNow = linkCode(iLink);
                minDist = 1e2;
                minDistObs = 0;
                for iObs = 1:nObs
                    [contactDist, ~, ~, ~, ~, ~] = ...
                        controller.robotModel.detectContact(controller.obstacleList{iObs},qNow,linkCodeNow);
                    if minDist > contactDist
                        minDist = contactDist;
                        minDistObs = iObs;
                    end
                end
                % Now compute the relevant variables for the closest
                % obstacle of the respective robot link.
                [contactDist, ~, ~, contactNormal, ~, contactTransJacobianGeometric] = ...
                    controller.robotModel.detectContact(controller.obstacleList{minDistObs},qNow,linkCodeNow);
                contactDistMtx(1, iLink) = contactDist;
                contactNormalMtx(1:3, iLink) = contactNormal;
                contactJacobMtx(1:3,1:linkCodeNow,iLink) = contactTransJacobianGeometric;
            end

            % compute the desired joint rate
            velToGoalPadded = [velToGoal; zeros(3, 1)];
            Jg = controller.robotModel.kinematic_extra.get_geometric_jacobian(qNow);

            % reorganizing Jg such that top three rows are for linear
            % and bottom three rows are for angular velocities
            Jg_corrected = [Jg(4:6, :); Jg(1:3, :)];
            ip_task2js = beta * Jg_corrected' * ((Jg_corrected*Jg_corrected') \ (velToGoalPadded));

            % Set the global variables for PATH solver to use
            setGlobalVariables(contactJacobMtx, contactNormalMtx, contactDistMtx, ip_task2js, qNow, length(linkCode));

            % call the PATH solver
            l = repelem(-Inf,11);
            u = repelem(Inf,11);
            l(1, 8:11) = 0;   % since complementarity velocity is always >= 0 
            
            [controller.z, ~, ~] = pathmcp(controller.z, l, u, 'mcpfuncjacEval2_mex');
            controller.q = controller.z(1:7, 1);
            controller.tick = toc(ht);

            % update the status
            controller.stepSend();
            controller.stepUpdateCounter();
        end
    end
end
