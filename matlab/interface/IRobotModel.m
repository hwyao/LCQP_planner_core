% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef IRobotModel < handle
    %IROBOTMODEL The Interface of the robot model
    
    properties (Abstract)
        kinematic
        % the kinematic structure of the robot

        vrep
        % the vrep interface for communication
    end

    methods (Abstract)
        [contactDist, contactPtObs, contactPtRobot, contactNormal, contactJacobian] = detectContact(model,obstacle,iLink);
        % DETECTCONTACT detect the basic contact information bewteen the robot model and
        % the obstacle.
        % Input:
        %    obstacle: the implementation of IObstacle, reprensenting an
        %     object in workspace that can have contact with the robot
        %     model.
        %    iLink: the link number of robot. It could be between value
        % Output:
        %    contactDist: the shortest distance before the contact 
        %    contactPtObstacle: the point of shortest contact on obstacle
        %    contactPtRobot: the point of shortest contact on robot
        %    contactNormal: the normal vector pointing outward from the
        %     contact point.
        %    contactJacobian: the Jacobian of the contact point.

        x = fkm(model,q);
        % FKM calculate the forward kinematic of the robot with a specific
        % configuration.
        % Input:
        %   q: the joint configuration.
        % Outpu:
        %   x: the position represented in DQ.

        J = poseJacobian(model,q)
        % POSEJACOBIAN calculate the pose jacobian of the robot with a specific
        % configuration.
        % Input:
        %   q: the joint configuration.
        % Outpu:
        %   x: the Jacobian matrix (for DQ).

        q = fetchStatus(model)
        % FETCHSTATUS fetch the joint configuration of the robot

        updateStatus(model,q)
        % UPDATESTATUS update the joint configuration of the robot
    end
end

