% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef IController < handle
    %IController the Interface of the controller
    
    properties (Abstract)
        %%%%%%%%%%%%%% handle properties %%%%%%%%%%%%%%
        robotModel IRobotModel
        % the class that represent the robot model

        obstacleList cell {mustBeIObstacleCell(obstacleList)}
        % the list of obstacle.

        %%%%%%%%%%%%%% simulation properties %%%%%%%%%%%%%%
        dt(1,1) double
        % the time of each step
        
        toleranceEnd(1,1) double 
        % the tolerance that considers that the robot reaches the target

        maxEndCount(1,1) double
        % the number of step that robot should stay in "toleranceEnd" to end the planning 
        
        maxStep(1,1) double
        % the maximum step that we can take to force endless running

        %%%%%%%%%%%%% simulation status %%%%%%%%%%%%%%%%%%%%%
        q(:,1) double
        % the current joint configuration

        currentEndCount(1,1) double
        % the current count of robot stay in the condition of ending

        currentStep(1,1) double
        % the current step of the
    end
    
    methods (Abstract)
        isReached = goalReached(controller)
        % GOALREACHED check if the goal is reached

        nextStep(controller)
        % NEXTSTEP check the status of the controller step and update the
        % next step
    end

    methods
        function isEnd = checkEnd(controller)
        % CHECKEND check of the robot is reaching the end.
        % Always called 1:1 with each nextStep() callup
            isEnd = false;
            if controller.currentEndCount >= controller.maxEndCount || ...
               controller.currentStep > controller.maxStep
                isEnd = true;
            end
        end

        function startSimulation(controller)
        % STARTSIMULATION start the vrep simulation
            controller.robotModel.dqVrep.disconnect_all();
            controller.robotModel.dqVrep.connect('127.0.0.1',19997);
            controller.robotModel.dqVrep.start_simulation();
        end

        function stopSimulation(controller)
        % TERMINATESIMULATION end the vrep simulation
            controller.robotModel.dqVrep.stop_simulation();
            controller.robotModel.dqVrep.disconnect();
        end
    end

    methods (Access=protected)
        function sendAndStep(controller)
        % SENDANDSTEP send the current Q, increase a step for all the
        % status
            controller.robotModel.updateStatus(controller.q);
            controller.currentStep = controller.currentStep + 1;
            if controller.goalReached() == true
                controller.currentEndCount = controller.currentEndCount + 1;
            else
                controller.currentEndCount = 0;
            end
        end
    end
end

function mustBeIObstacleCell(obstacleList)
    cellfun(@(x)mustBeA(x, 'IObstacle'),obstacleList)
end