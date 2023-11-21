%% Object initialization
% clear the variables
clear
clc

% Initialize the Vrep model
include_namespace_dq
dqVrep = DQ_VrepInterface;
dqVrep.disconnect_all();
dqVrep.connect('127.0.0.1',19997);

% initialize the RobotModel
robotModel = RobotModelFrankaBar("Franka",dqVrep);

% initialize the obstacle
% obstacle1 = ObstacleSphere("/Sphere[0]",true,dqVrep);
% obstacle1.radius = 0.05;
% obstacle2 = ObstacleSphere("/Sphere[1]",false,dqVrep);
% obstacle2.radius = 0.1;
% obstacle3 = ObstacleSphere("/Sphere[2]",true,dqVrep);
% obstacle3.radius = 0.05;
% obstacleList = {obstacle1,obstacle2,obstacle3};
% obstacleList = {};


% initialize one obstacle
obstacle1 = ObstacleSphere("/Sphere[0]",false,dqVrep);
obstacle1.radius = 0.05;

obstacleList = {obstacle1};
%obstacleList = {};


%% Planning initialization
% set the planning goal
goal = [0.4,0.3,0.6];
tau = 0.01;

% initialize the controller
% controller = ControllerLCQP(robotModel,obstacleList,goal);
controller = ControllerFeasibility(robotModel,obstacleList,goal);

%% Planning loop
controller.startSimulation();
qNow = controller.q;
fkNow = controller.robotModel.fkm(qNow);
posDQ = DQ(goal);
rotNow = fkNow.rotation;
% construct DQ for goal
dqGoal = rotNow + E_ * 0.5 * posDQ * rotNow;
norm_dqGoal = dqGoal * inv(norm(dqGoal));

obsPosList = zeros(1,3);
%obsPosList = [];
% update the initial value (if needed)
% qStart = [];
% controller.robotModel.updateStatus(qStart);
while controller.checkEnd() == false
    % get current joints
    q_curr = controller.q;  
    fkNowc = controller.robotModel.fkm(q_curr);
    posDQc = fkNowc.translation;
    rotNowc = fkNowc.rotation;
    % construct DQ for goal
    dq_c = rotNowc + E_ * 0.5 * posDQc * rotNowc;
    norm_dq_c = dq_c * inv(norm(dq_c));
    % perform screw interpolation 
    M = Sclerp(norm_dq_c,norm_dqGoal,tau);
   % push the controller to next step
    controller.nextStep(obsPosList);
    controller.goal = DQ(M(:,2)).translation.vec3;
    %controller.q
    %controller.robotModel.fkm(controller.q).translation.q(2:4)
end




% % some obstacle controll
% phase = 1;
% posObs1 = [-0.15,0.15,0.6];
% posObs3 = [-0.15,-0.55,0.6];
% 
% while controller.checkEnd() == false
%     % inject the external data
%     if phase == 1
%         posObs1 = posObs1 + [0,0.005,0];
%         posObs3 = posObs3 + [0,0.005,0];
%         if posObs1(2) >= 0.65
%             phase = 2;
%         end
%     elseif phase == 2
%         posObs1 = posObs1 - [0,0.005,0];
%         posObs3 = posObs3 - [0,0.005,0];
%         if posObs1(2) <= 0.05
%             phase = 3;
%         end
%     end
% 
%     obsPosList = zeros(3,3);   % for passive obstacle, this is useless but we should have a dummy input.
%     obsPosList(1,:) = posObs1;
%     obsPosList(3,:) = posObs3;
% 
%     % push the controller to next step
%     controller.nextStep(obsPosList);
% end
controller.stopSimulation();