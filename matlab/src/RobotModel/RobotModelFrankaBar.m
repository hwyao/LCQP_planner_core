%ROBOTMODELBAR The franka Emika robot model with bar convention

% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef RobotModelFrankaBar < IRobotModel
    properties (SetAccess=immutable)
        DH =    [0,      0,      0,      0,      0,          0,      0;
                 0.333,  0,      0.316,  0,      0.384,      0,      0;
                 0,      0,      0,      0.0825, -0.0825,    0,      0.088;
                 0,      -pi/2,  pi/2,   pi/2,   -pi/2,      pi/2,   pi/2];

        EE =    1 + DQ.E*0.5*DQ.k*0.107;

        jointName = ["Franka_joint1","Franka_joint2","Franka_joint3","Franka_joint4",...
                     "Franka_joint5","Franka_joint6","Franka_joint7"];

        jointVrep; 
    end

    properties 
        kinematic = DQ_SerialManipulator.empty;

        kinematic_extra FrankaKinematic = FrankaKinematic.empty;

        dqVrep = DQ_VrepInterface.empty;
    end
    
    methods
        function model = RobotModelFrankaBar(robotName,dqVrep)
            arguments
                robotName(1,1) string
                dqVrep(1,1)
            end

            modelName = subsref(split(robotName,"#"),struct('type', '()', 'subs', {{1}}));
            if ~isequal("FrankaFix",modelName) && ~isequal("Franka",modelName)
                error("Please input a Franka model.");
            end
            jointVrepName = "/" + modelName + "/" + model.jointName; 
            
            model.jointVrep = num2cell(char(jointVrepName(:)),2);
            model.kinematic = DQ_SerialManipulator(model.DH,'modified');
            model.kinematic.set_effector(model.EE);
            model.kinematic_extra = FrankaKinematic(model.DH,model.EE);
            model.dqVrep = dqVrep;
        end
    end

    methods
        function [contactDist, contactPtObs, contactPtRobot, ...
                  contactNormal, contactTransJacobian, contactTransJacobianGeometric] = detectContact(model,obstacle,q,iLink)
            objectClassName = class(obstacle);
            if isequal(objectClassName,"ObstacleSphere")
                [contactDist, contactPtObs, contactPtRobot, ...
                 contactNormal, contactTransJacobian, contactTransJacobianGeometric] = barSphereContact(model,obstacle,q,iLink);
            end
        end
    end
end

