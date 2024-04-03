%% Object initialization
% clear the variables
clear
clc

% Initialize the Vrep model
dqVrep = DQ_VrepInterface;
dqVrep.disconnect_all();
dqVrep.connect('127.0.0.1',19997);

% initialize the RobotModel
% this model a modified version by https://github.com/hwyao/CoppeliaSim_Franka_ModelFix
robotModel = RobotModelFrankaBar("FrankaFix",dqVrep);

% initialize the obstacle
obstacle1 = ObstacleSphere("/Sphere[0]",false,dqVrep);
obstacle1.radius = 0.05;
obstacle2 = ObstacleSphere("/Sphere[1]",false,dqVrep);
obstacle2.radius = 0.05;
obstacle3 = ObstacleSphere("/Sphere[2]",false,dqVrep);
obstacle3.radius = 0.05;
obstacle4 = ObstacleSphere("/Sphere[3]",false,dqVrep);
obstacle4.radius = 0.05;
obstacle5 = ObstacleSphere("/Sphere[4]",false,dqVrep);
obstacle5.radius = 0.05;
obstacle6 = ObstacleSphere("/Sphere[5]",false,dqVrep);
obstacle6.radius = 0.05;
obstacleList = {obstacle1,obstacle2,obstacle3,obstacle4,obstacle5,obstacle6};
%obstacleList = {};

% set the planning goal
%goal = [0.5,0.4,0.6];
%goal = [0.325,0.35,0.65];
goal = [0.05,0.55,0.40];

% set the controller
controller = ControllerLCQP(robotModel,obstacleList,goal);

%% Planning loop
controller.startSimulation();

% update the initial value (if needed)
% qStart = [];
% controller.robotModel.updateStatus(qStart);

% some obstacle controll
phase = 1;
posObs1 = [0.4,0.5,0.4];
posObs2 = [-0.13,-0.52,0.5];
posObs3 = [0.2,0.5,0.4];

while controller.checkEnd() == false
    % inject the external data
    if phase == 1
        %posObs1 = posObs1 + [0,-0.005,0];
        posObs2 = posObs2 + [0,0.005,0];
        %posObs3 = posObs3 + [0,-0.005,0];
        if posObs2(2)>= - 0.15
            phase = 2;
        end
    else
        posObs1 = posObs1 + [0,-0.0003,0];
        posObs2 = posObs2 + [0,-0.005,0];
        posObs3 = posObs3 + [0,-0.0003,0];
    end


    %obsPosList = zeros(3,3);   % for passive obstacle, this is useless but we should have a dummy input.
    obsPosList(1,:) = posObs1;
    obsPosList(2,:) = posObs2;
    obsPosList(3,:) = posObs3;
    obsPosList(4,:) = posObs3;
    obsPosList(5,:) = posObs3;
    obsPosList(6,:) = posObs3;

    % push the controller to next step
    controller.nextStep(obsPosList);
end

controller.stopSimulation();