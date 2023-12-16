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

        data

        dataSize

        tol = 1e-5
    end
    
    methods(TestClassSetup)
        function setup(testCase)
            vi = DQ_VrepInterface;
            testCase.robot = RobotModelFrankaBar("FrankaFix",vi);
            testCase.obstacle = ObstacleSphere("/Sphere",false,vi);
            testCase.data = load(matlab.project.rootProject().ProjectStartupFolder+"/matlab/data/test_data");
            testCase.dataSize = numel(testCase.data.test_data);
        end
    end
    
    methods(Test) 
        function test(testCase)
            for iTest = 1:testCase.dataSize
                testCase.obstacle.center = testCase.data.test_data(iTest).obs_pose;
                testCase.obstacle.radius = testCase.data.test_data(iTest).obs_radius;
                q = testCase.data.test_data(iTest).q;
                iLink = testCase.data.test_data(iTest).link;

                contactDistWanted = testCase.data.test_data(iTest).distance;
                contactPtObsWanted = testCase.data.test_data(iTest).spherePoint;
                contactPtRobotWanted = testCase.data.test_data(iTest).closestPoint;

                [contactDist, contactPtObs, contactPtRobot, ...
                 ~, ~] = barSphereContact(testCase.robot,testCase.obstacle,q,iLink);
    
                testCase.assertEqual(contactDist,contactDistWanted,'AbsTol',testCase.tol);
                testCase.assertEqual(contactPtObs,contactPtObsWanted,'AbsTol',testCase.tol);
                testCase.assertEqual(contactPtRobot,contactPtRobotWanted,'AbsTol',testCase.tol);
            end
        end
    end
    
end