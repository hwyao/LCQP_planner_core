% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal
%   circulation. Any modification, distribution, private or commercial use
%   outside the repository cooperation without the consent of all contributors
%   is strictly forbidden.
%
% Contributor: Haowen Yao
classdef IRobotModel < handle
    %IROBOTMODEL The Interface of the robot model
    
    properties (Abstract,SetAccess = immutable)
        DH;
        % the DH parameters of the robot model
        
        EE;
        % the EE parameters of the robot model
        
        jointName;
        % the name of each joint (without the prefix of the robot name)
        
        jointVrep;
        % the name of each joint for communication to vrep
    end
    
    properties (Abstract)
        kinematic DQ_SerialManipulator
        % the kinematic structure of the robot
        
        dqVrep DQ_VrepInterface
        % the DQ vrep interface for communication
    end
    
    methods (Abstract)
        [contactDist, contactPtObs, contactPtRobot, contactNormal, contactTransJacobian] = detectContact(model,obstacle,iLink);
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
    end
    
    methods
        function x = fkm(model,q)
            % FKM calculate the forward kinematic of the robot with a specific
            % configuration.
            % Input:
            %   q: the joint configuration.
            % Outpu:
            %   x: the position represented in DQ.
            x = model.kinematic.fkm(q);
        end
        
        function J = poseJacobian(model,q)
            % POSEJACOBIAN calculate the pose jacobian of the robot with a specific
            % configuration.
            % Input:
            %   q: the joint configuration.
            % Outpu:
            %   x: the Jacobian matrix (for DQ).
            J = model.kinematic.pose_jacobian(q);
        end
        
        function q = fetchStatus(model)
            % FETCHSTATUS fetch the joint configuration of the robot
            q = model.dqVrep.get_joint_positions(model.jointVrep);
        end
        
        function updateStatus(model,q)
            % UPDATESTATUS update the joint configuration of the robot
            model.dqVrep.set_joint_positions(model.jointVrep,q)
        end
    end
end

