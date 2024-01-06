% IOBSTACLE the Interface of the obstacle

% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef IObstacle < handle
    properties (Abstract, SetAccess = immutable)
        % the name of the obstacle in vrep
        name(1,:) char
        
        % indicate the active/passive status of the obstacle
        % active(true): We send the position status to control the obstacle
        % passive(false): We get the position status from vrep
        isActive(1,1) logical
    end
    
    properties (Abstract)
        % the DQ vrep interface of the obstacle
        dqVrep DQ_VrepInterface 
    end

    methods
        % update the status of the obstacle
        updateStatus(obs)
    end
end