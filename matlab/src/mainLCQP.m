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
obstacle1 = ObstacleSphere("/Sphere[0]",true,dqVrep);
obstacle1.radius = 0.05;
obstacle2 = ObstacleSphere("/Sphere[1]",false,dqVrep);
obstacle2.radius = 0.1;
obstacle3 = ObstacleSphere("/Sphere[2]",true,dqVrep);
obstacle3.radius = 0.05;
obstacleList = {obstacle1,obstacle2,obstacle3};

% set the planning goal
goal = [0.5,0.4,0.6];

% set the controller
controller = ControllerLCQP(robotModel,obstacleList,goal);

%% Planning loop
controller.startSimulation();

% update the initial value (if needed)
% qStart = [];
% controller.robotModel.updateStatus(qStart);

% some obstacle controll
phase = 1;
posObs1 = [-0.15,0.15,0.6];
posObs3 = [-0.15,-0.55,0.6];

while controller.checkEnd() == false
    % inject the external data
    if phase == 1
        posObs1 = posObs1 + [0,0.005,0];
        posObs3 = posObs3 + [0,0.005,0];
        if posObs1(2) >= 0.65
            phase = 2;
        end
    elseif phase == 2
        posObs1 = posObs1 - [0,0.005,0];
        posObs3 = posObs3 - [0,0.005,0];
        if posObs1(2) <= 0.05
            phase = 3;
        end
    end

    obsPosList = zeros(3,3);   % for passive obstacle, this is useless but we should have a dummy input.
    obsPosList(1,:) = posObs1;
    obsPosList(3,:) = posObs3;

    % push the controller to next step
    controller.nextStep(obsPosList);
end
controller.stopSimulation();