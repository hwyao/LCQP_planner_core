% prepare start and goal
% config = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]';
% goal = [0.05,0.55,0.60];

% prepare data (scene 1)
positionList = [0.4,0.5,0.4;
                -0.13,-0.52,0.5;
                0.2,0.5,0.4];
radiusList = [0.05;0.1;0.05];
[isValid,config,goal,configGoal,worldObjects] = generateStartGoal(positionList,radiusList);
if ~isValid
    error("Set point invalid!");
end

% LCQP planner
tLCQP = TaskLCQP();
tLCQP.init(positionList,radiusList,config,goal);
tLCQP.run();
tblLCQP = tLCQP.dataTable;

% LCQPath planner
% setGlobalVariablesOnce(7,1e-3,1e-1);
% tLCPath = TaskLCPath();
% tLCPath.init(positionList,radiusList,config,goal);
% tLCPath.run();
% tblLCPath = tLCPath.dataTable;

% variables for non-realtime planner
t = tblLCQP.time(end);
dt = tLCQP.controller.dt;

% CHOMP planner
tCHOMP = TaskCHOMP(t,dt);
tCHOMP.init(positionList,radiusList,config,configGoal);
tCHOMP.run();
tblCHOMP = tCHOMP.dataTable;

% RRT planner
tRRT = TaskRRT(t,dt);
tRRT.init(positionList,radiusList,config,configGoal);
tRRT.run();
tblRRT = tRRT.dataTable;

% These table files could be saved later just for data callup
tableCompare.tblLCQP = tblLCQP;
tableCompare.tblCHOMP = tblCHOMP;
tableCompare.tblRRT = tblRRT;
tableCompare.configStart = config;
tableCompare.configGoal = configGoal;
tableCompare.goal = goal;
tableCompare.worldObjects = worldObjects;
tableCompare.saveTime = datetime('now','Format','yyyyMMdd_HHmmss');

% output the result comparison
% [pathLength,averageJointMovement] = reportTask(tblLCQP)