% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
classdef testBallSphereContact < matlab.unittest.TestCase
    
    properties
        robot
        
        obstacle
    end
    
    methods(TestClassSetup)
        function setup(testCase)
            vi = DQ_VrepInterface;
            testCase.robot = RobotModelFrankaBar("FrankaFixed",vi);
            testCase.obstacle = ObstacleSphere("/Sphere",false,vi);
        end
    end
    
    methods(Test) 
        function test(testCase)
            testCase.obstacle.center = [0;0;0.1];
            testCase.obstacle.radius = 0.05;
            
            q = [0;0;0;0;0;0;0];
            iLink = 1;

            [contactDist, contactPtObs, contactPtRobot, ...
             contactNormal, ~] = barSphereContact(testCase.robot,testCase.obstacle,q,iLink);

            testCase.assumeEqual(contactDist,0);
            testCase.assumeEqual(contactPtObs,[0;0;0]);
            testCase.assumeEqual(contactPtRobot,[0;0;0]);
            testCase.assumeEqual(contactNormal,[0;0;0]);
        end
    end
    
end