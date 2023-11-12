% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef IObstacle < handle
    %IOBSTACLE the Interface of the obstacle
    
    properties (Abstract)
        center
        % the center point of the obstacle

        isActive
        % indicate the active/passive status of our program
        % active: We send the position status to control the obstacle
        % passive: We get the position status from vrep

        vrep
        % the vrep interface of the obstacle
    end
end