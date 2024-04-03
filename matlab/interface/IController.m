% IController the Interface of all controllers

% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef IController < handle
    properties (Abstract)
        %%%%%%%%%%%%%% handle properties %%%%%%%%%%%%%%

        % class handle that represent the robot model
        robotModel IRobotModel
        
        % list of handles that contains information of obstacle.
        obstacleList cell {mustBeIObstacleCell(obstacleList)}

        %%%%%%%%%%%%%% simulation properties %%%%%%%%%%%%%%
        
        % time of each step
        dt(1,1) double
        
        % tolerance that considers that the robot reaches the target
        % See: goalReached
        toleranceEnd(1,1) double 
        
        % number of step that robot should stay in toleranceEnd to end the planning 
        % See: currentEndCount, stepUpdateCounter, checkEnd
        maxEndCount(1,1) double
        
        % maximum iterations of step that we can take to interrupt endless running
        % See: currentStep, stepUpdateCounter, checkEnd
        maxStep(1,1) double

        %%%%%%%%%%%%% simulation status %%%%%%%%%%%%%%%%%%%%%
        
        % current joint configuration
        q(:,1) double

        % the time taken for each iteration
        tick(1,1) double
        
        % current count of robot stay in the condition of ending
        % See: goalReached, stepUpdateCounter, checkEnd
        currentEndCount(1,1) double
        
        % current total step of the controller running 
        % See: stepUpdateCounter, checkEnd
        currentStep(1,1) double
    end
    
    methods (Abstract)
        % GOALREACHED check if the goal is reached. 
        % called by stepUpdateCounter to decide if currentEndCount
        % should be increased. Return true to represent goal reached.
        % See: stepUpdateCounter, currentEndCount
        isReached = goalReached(controller)
         
        % NEXTSTEP update the controller and step once forward.
        % The implementaiton of controller should be here.
        % Consider to call some function in start and end of nextStep to
        % update the status.
        % See: stepSend, stepUpdateCounter
        nextStep(controller)
    end

    methods
        function isEnd = checkEnd(controller)
        % CHECKEND check of the robot is reaching the end.
        % Called externally to notify if 
        % Suggest to be called 1:1 parallel with each nextStep callup. 
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
        function stepSend(controller)
        % STEPSEND send the joint configuration. Should be called in each
        % nextStep() iteration of realtime plannner.
            controller.robotModel.updateStatus(controller.q);
        end

        function stepUpdateCounter(controller)
        % STEPUPDATECOUNTER update the iteration counter. Should be called 
        % in each nextStep() iteration.
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