% Contributor: Anirban Sinha
classdef ControllerSimple < IController    
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
        z
        loop_cnt = 0

        %%%%%%%%%%%%% simulation goal %%%%%%%%%%%%%%%%%%%%%%%
        goal(3,1) double
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

        function nextStep(controller,obsPosList)
            cMainTask = 0.5;
            hContact = 0.001;
            % get coordiate status
            qNow = controller.q;
            fkNow = controller.robotModel.fkm(qNow);
            posNow = fkNow.translation.vec3;
            posGoal = controller.goal;
            velToGoal = cMainTask * (posGoal - posNow) / norm(posGoal - posNow);

            % compute the desired joint rate
            velToGoalPadded = [velToGoal; zeros(3, 1)];
            Jg = geomJ(controller.robotModel.kinematic, qNow);
            % % % Jg_corrected = Jg;
            Jg_corrected = [Jg(4:6, :); Jg(1:3, :)];
            qdot = Jg_corrected' * ((Jg_corrected*Jg_corrected') \ (velToGoalPadded));

            % % % controller.xLast = controller.z(1:7, 1);
            % % % qdot = controller.xLast;
            controller.q = controller.q + hContact * qdot;
            % % % controller.q = controller.q + qdot; 

            % update the status
            controller.stepSend();
            controller.stepUpdateCounter();

            % increase the loop count
            controller.loop_cnt = controller.loop_cnt + 1;
            fprintf("simulation step: %d\n", controller.loop_cnt);
        end
    end
end
