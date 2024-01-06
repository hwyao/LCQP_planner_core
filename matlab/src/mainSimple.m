%% Object initialization
% clear the variables
clc;
clear;

% Initialize the Vrep model
dqVrep = DQ_VrepInterface;
dqVrep.disconnect_all();
dqVrep.connect('127.0.0.1',19997);

% initialize the RobotModel
% this model a modified version by https://github.com/hwyao/CoppeliaSim_Franka_ModelFix
robotModel = RobotModelFrankaBar("FrankaFix",dqVrep);

%% Planning initialization
% set the planning goal
goal = [0.5,0.4,0.6];

% initialize the controller
controller = ControllerSimple(robotModel,goal);

%% Planning loop
controller.startSimulation();

while controller.checkEnd() == false
    % push the controller to next step
    controller.nextStep();
end

controller.stopSimulation();