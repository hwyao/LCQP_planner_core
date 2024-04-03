function tableCompare = taskCompare(positionList,radiusList)
    [isValid,config,goal,configGoal,worldObjects] = generateStartGoal(positionList,radiusList);
    if ~isValid
        error("Set point invalid!");
    end

    disp("config:"+mat2str(config'));
    disp("goal:"+mat2str(goal));
    disp("configGoal:"+mat2str(configGoal'));
    
    % gcp
    if gcp().Busy == true
        disp("Warning: Something might be wrong, gcp busying");
    end

    % LCQP planner
    tLCQP = TaskLCQP();
    % tLCQP.init(positionList,radiusList,config,goal);
    % tLCQP.run();
    % tblLCQP = tLCQP.dataTable;
    tblLCQP = runTask(tLCQP,positionList,radiusList,config,goal);
    
    % variables for non-realtime planner
    t = tblLCQP.time(end);
    dt = tLCQP.controller.dt;
    
    % CHOMP planner
    tCHOMP = TaskCHOMP(t,dt);
    % tCHOMP.init(positionList,radiusList,config,configGoal);
    % tCHOMP.run();
    % tblCHOMP = tCHOMP.dataTable;
    % tblCHOMP = runTask(tCHOMP,positionList,radiusList,config,configGoal);
    FCHOMP = parfeval(@runTask,1,tCHOMP,positionList,radiusList,config,configGoal);
    
    % RRT planner
    tRRT = TaskRRT(t,dt);
    % tRRT.init(positionList,radiusList,config,configGoal);
    % tRRT.run();
    % tblRRT = tRRT.dataTable;
    %tblRRT = runTask(tRRT,positionList,radiusList,config,configGoal);
    FRRT = parfeval(@runTask,1,tRRT,positionList,radiusList,config,configGoal);

    wait([FRRT,FCHOMP],"finished",20);
    if isequal(FCHOMP.State,"finished")
        tblCHOMP = fetchOutputs(FCHOMP);
        disp("CHOMP:finished");
    else
        tblCHOMP = table();
        cancel(FCHOMP);
        disp("CHOMP:calcelled");
    end
    if isequal(FRRT.State,"finished")
        tblRRT = fetchOutputs(FRRT);
        disp("RRT:finished");
    else
        tblRRT = table();
        cancel(FRRT);
        disp("RRT:calcelled");
    end
    
    % These table files could be saved later just for data callup
    tableCompare.tblLCQP = tblLCQP;
    tableCompare.tblCHOMP = tblCHOMP;
    tableCompare.tblRRT = tblRRT;
    tableCompare.configStart = config;
    tableCompare.configGoal = configGoal;
    tableCompare.goal = goal;
    tableCompare.worldObjects = worldObjects;
    tableCompare.saveTime = datetime('now','Format','yyyyMMdd_HHmmss');
end

