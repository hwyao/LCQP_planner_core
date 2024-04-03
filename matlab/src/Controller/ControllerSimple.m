% CCONTROLLERSIMPLE the simple CLIK planner without collision avoidance.

% Contributor: Anirban Sinha
classdef ControllerSimple < IController    
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
        % Goal of the planner. Here we accept the goal as position.
        goal(3,1) double

        % constant for main task c*J*dx
        constMainTask = 0.0005;
    end
    
    methods
        function controller = ControllerSimple(robotModel,goal)
            controller.robotModel = robotModel;
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

        function nextStep(controller)

            % get constants
            cMainTask = controller.constMainTask;

            % get coordiate status
            qNow = controller.q;
            fkNow = controller.robotModel.fkm(qNow);
            posNow = fkNow.translation.vec3;
            posGoal = controller.goal;
            velToGoal = cMainTask * (posGoal - posNow) / norm(posGoal - posNow);

            % compute the desired joint rate
            velToGoalPadded = [velToGoal; zeros(3, 1)];
            Jg = controller.robotModel.kinematic_extra.get_geometric_jacobian(qNow);
            Jg_corrected = [Jg(4:6, :); Jg(1:3, :)];
            qdot = Jg_corrected' * ((Jg_corrected*Jg_corrected') \ (velToGoalPadded));

            % update the joint configuration
            controller.q = controller.q + qdot;

            % update the status
            controller.stepSend();
            controller.stepUpdateCounter();
        end
    end
end
