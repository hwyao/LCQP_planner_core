% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef IObstacle < handle
    %IOBSTACLE the Interface of the obstacle
    
    properties (Abstract, SetAccess = immutable)
        name(1,:) char
        % the name of the obstacle in vrep

        isActive(1,1) logical
        % indicate the active/passive status of our program
        % active: We send the position status to control the obstacle
        % passive: We get the position status from vrep
    end
    
    properties (Abstract)
        dqVrep DQ_VrepInterface 
        % the DQ vrep interface of the obstacle
    end

    methods
        updateStatus(obs)
        % update the status of the obstacle
    end
end