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
        currentEndCount = 0
        currentStep = 0
    end

    % planner specific properties
    properties
        goal(3,1) double

        constMainTask = 0.005;
    end
    
    methods
        function controller = ControllerSimple(robotModel,obstacleList,goal)
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

        function nextStep(controller)
            cMainTask = controller.constMainTask;

            % get coordiate status
            qNow = controller.q;
            fkNow = controller.robotModel.fkm(qNow);
            posNow = fkNow.translation.vec3;
            posGoal = controller.goal;
            velToGoal = cMainTask * (posGoal - posNow) / norm(posGoal - posNow);

            % compute the desired joint rate
            velToGoalPadded = [velToGoal; zeros(3, 1)];
            Jg = geomJ(controller.robotModel.kinematic, qNow);
            Jg_corrected = [Jg(4:6, :); Jg(1:3, :)];
            qdot = Jg_corrected' * ((Jg_corrected*Jg_corrected') \ (velToGoalPadded));

            % update the joint configuration
            controller.q = controller.q + hContact * qdot;

            % update the status
            controller.stepSend();
            controller.stepUpdateCounter();
        end
    end
end
