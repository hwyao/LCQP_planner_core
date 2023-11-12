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
        robotModel
        % the class that represent the robot model

        obstacleList
        % the list of obstacle.

        toleranceEnd
        % the tolerance that considers that the robot reaches the target

        countEnd
        % the number of step that robot should stay in "toleranceEnd" to end the planning 

        countCurrent
        % the current count of robot stay in the "toleranceEnd"
    end
    
    methods
        isEnd = checkEnd(controller)
        % CHECKEND check of the robot is reaching the end.
        % Always called 1:1 with each nextStep() callup

        nextStep(controller)
        % NEXTSTEP check the status of the controller step and update the
        % next step
    end
end

