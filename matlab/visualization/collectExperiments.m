list = ls("./matlab/data");
listCell = split(list,[newline," ",char(9)]);
listCell = listCell(strlength(listCell) > 0);

listCellscene1 = {};
listCellscene2 = {};
listCellscene3 = {};
listCellscene4 = {};
listCellscene5 = {};

for iCell = 1:numel(listCell)
    result = listCell{iCell};
    if contains(result,'scene1-')
        listCellscene1 = [listCellscene1,{result}];  %#ok<*AGROW>
    elseif contains(result,'scene2-')
        listCellscene2 = [listCellscene2,{result}];
    elseif contains(result,'scene3-')
        listCellscene3 = [listCellscene3,{result}];
    elseif contains(result,'scene4-')
        listCellscene4 = [listCellscene4,{result}];
    elseif contains(result,'scene5-')
        listCellscene5 = [listCellscene5,{result}];
    end
end

rbt = loadrobot("frankaEmikaPanda","DataFormat","column");
rbt.removeBody("panda_hand");


%%%%%%%%%%%%%%%%%%%%%%%% S1
successLCQP = 0;
subsuccessLCQP = 0;
successCHOMP = 0;
successRRT = 0;
plLCQP = 0;
plCHOMP = 0; 
plRRT = 0;
avgLCQP = 0;
avgCHOMP = 0;
avgRRT = 0;
worldObjects = {};
load("./matlab/data/scene1meta.mat");
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
for iCell = 1:numel(listCellscene1)
    load("./matlab/data/"+listCellscene1{iCell});
    if norm(tableCompare.goal - tableCompare.tblLCQP.x{end}') <= 0.01  
        qCell = tableCompare.tblLCQP.q;
        isCollide = false;
        for iq = 1:numel(qCell)
            [isCollide,~,~] = checkCollision(rbt,qCell{iq},worldObjects,SkippedSelfCollisions="parent");
            if isCollide
                subsuccessLCQP = subsuccessLCQP + 1;
                break;
            end
        end
        if ~isCollide
            successLCQP = successLCQP + 1;
            [pl,avg] = reportTask(tableCompare.tblLCQP);
            plLCQP = plLCQP + pl;
            avgLCQP = avgLCQP + avg;
        end
    end
    
    if ~isempty(tableCompare.tblCHOMP)
        successCHOMP = successCHOMP + 1;
        [pl,avg] = reportTask(tableCompare.tblCHOMP);
        plCHOMP = plCHOMP + pl;
        avgCHOMP = avgCHOMP + avg;
    end

    if ~isempty(tableCompare.tblRRT)
        successRRT = successRRT + 1;
        [pl,avg] = reportTask(tableCompare.tblRRT);
        plRRT = plRRT + pl;
        avgRRT = avgRRT + avg;
    end
end
plLCQP = plLCQP / successLCQP;
avgLCQP = avgLCQP / successLCQP;
plCHOMP = plCHOMP / successCHOMP;
avgCHOMP = avgCHOMP / successCHOMP;
plRRT = plRRT / successRRT;
avgRRT = avgRRT / successRRT;
disp("Scenario 1: success "+successLCQP+"("+subsuccessLCQP+") "+successCHOMP+" "+successRRT+" "+numel(listCellscene1))
disp("Scenario 1: pathLength "+plLCQP+" "+plCHOMP+" "+plRRT)
disp("Scenario 1: avgJointMovement "+avgLCQP+" "+avgCHOMP+" "+avgRRT)


%%%%%%%%%%%%%%%%%%%%%%%% S2
successLCQP = 0;
subsuccessLCQP = 0;
successCHOMP = 0;
successRRT = 0;
plLCQP = 0;
plCHOMP = 0; 
plRRT = 0;
avgLCQP = 0;
avgCHOMP = 0;
avgRRT = 0;
worldObjects = {};
load("./matlab/data/scene2meta.mat");
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
for iCell = 1:numel(listCellscene2)
    load("./matlab/data/"+listCellscene2{iCell});
    if norm(tableCompare.goal - tableCompare.tblLCQP.x{end}') <= 0.01  
        qCell = tableCompare.tblLCQP.q;
        isCollide = false;
        for iq = 1:numel(qCell)
            [isCollide,~,~] = checkCollision(rbt,qCell{iq},worldObjects,SkippedSelfCollisions="parent");
            if isCollide
                subsuccessLCQP = subsuccessLCQP + 1;
                break;
            end
        end
        if ~isCollide
            successLCQP = successLCQP + 1;
            [pl,avg] = reportTask(tableCompare.tblLCQP);
            plLCQP = plLCQP + pl;
            avgLCQP = avgLCQP + avg;
        end
    end
    
    if ~isempty(tableCompare.tblCHOMP)
        successCHOMP = successCHOMP + 1;
        [pl,avg] = reportTask(tableCompare.tblCHOMP);
        plCHOMP = plCHOMP + pl;
        avgCHOMP = avgCHOMP + avg;
    end

    if ~isempty(tableCompare.tblRRT)
        successRRT = successRRT + 1;
        [pl,avg] = reportTask(tableCompare.tblRRT);
        plRRT = plRRT + pl;
        avgRRT = avgRRT + avg;
    end
end
plLCQP = plLCQP / successLCQP;
avgLCQP = avgLCQP / successLCQP;
plCHOMP = plCHOMP / successCHOMP;
avgCHOMP = avgCHOMP / successCHOMP;
plRRT = plRRT / successRRT;
avgRRT = avgRRT / successRRT;
disp("Scenario 2: success "+successLCQP+"("+subsuccessLCQP+") "+successCHOMP+" "+successRRT+" "+numel(listCellscene1))
disp("Scenario 2: pathLength "+plLCQP+" "+plCHOMP+" "+plRRT)
disp("Scenario 2: avgJointMovement "+avgLCQP+" "+avgCHOMP+" "+avgRRT)

%%%%%%%%%%%%%%%%%%%%%%%%%%% S3
successLCQP = 0;
subsuccessLCQP = 0;
successCHOMP = 0;
successRRT = 0;
plLCQP = 0;
plCHOMP = 0; 
plRRT = 0;
avgLCQP = 0;
avgCHOMP = 0;
avgRRT = 0;
worldObjects = {};
load("./matlab/data/scene3meta.mat");
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
for iCell = 1:numel(listCellscene3)
    load("./matlab/data/"+listCellscene3{iCell});
    if norm(tableCompare.goal - tableCompare.tblLCQP.x{end}') <= 0.01  
        qCell = tableCompare.tblLCQP.q;
        isCollide = false;
        for iq = 1:numel(qCell)
            [isCollide,~,~] = checkCollision(rbt,qCell{iq},worldObjects,SkippedSelfCollisions="parent");
            if isCollide
                subsuccessLCQP = subsuccessLCQP + 1;
                break;
            end
        end
        if ~isCollide
            successLCQP = successLCQP + 1;
            [pl,avg] = reportTask(tableCompare.tblLCQP);
            plLCQP = plLCQP + pl;
            avgLCQP = avgLCQP + avg;
        end
    end
    
    if ~isempty(tableCompare.tblCHOMP)
        successCHOMP = successCHOMP + 1;
        [pl,avg] = reportTask(tableCompare.tblCHOMP);
        plCHOMP = plCHOMP + pl;
        avgCHOMP = avgCHOMP + avg;
    end

    if ~isempty(tableCompare.tblRRT)
        successRRT = successRRT + 1;
        [pl,avg] = reportTask(tableCompare.tblRRT);
        plRRT = plRRT + pl;
        avgRRT = avgRRT + avg;
    end
end
plLCQP = plLCQP / successLCQP;
avgLCQP = avgLCQP / successLCQP;
plCHOMP = plCHOMP / successCHOMP;
avgCHOMP = avgCHOMP / successCHOMP;
plRRT = plRRT / successRRT;
avgRRT = avgRRT / successRRT;
disp("Scenario 3: success "+successLCQP+"("+subsuccessLCQP+") "+successCHOMP+" "+successRRT+" "+numel(listCellscene1))
disp("Scenario 3: pathLength "+plLCQP+" "+plCHOMP+" "+plRRT)
disp("Scenario 3: avgJointMovement "+avgLCQP+" "+avgCHOMP+" "+avgRRT)

%%%%%%%%%%%%%%%%%%%%%%%%%% S4
successLCQP = 0;
subsuccessLCQP = 0;
successCHOMP = 0;
successRRT = 0;
plLCQP = 0;
plCHOMP = 0; 
plRRT = 0;
avgLCQP = 0;
avgCHOMP = 0;
avgRRT = 0;
worldObjects = {};
load("./matlab/data/scene4meta.mat");
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
for iCell = 1:numel(listCellscene4)
    load("./matlab/data/"+listCellscene4{iCell});
    if norm(tableCompare.goal - tableCompare.tblLCQP.x{end}') <= 0.01
        qCell = tableCompare.tblLCQP.q;
        isCollide = false;
        for iq = 1:numel(qCell)
            [isCollide,~,~] = checkCollision(rbt,qCell{iq},worldObjects,SkippedSelfCollisions="parent");
            if isCollide
                subsuccessLCQP = subsuccessLCQP + 1;
                break;
            end
        end
        if ~isCollide
            successLCQP = successLCQP + 1;
            [pl,avg] = reportTask(tableCompare.tblLCQP);
            plLCQP = plLCQP + pl;
            avgLCQP = avgLCQP + avg;
        end
    end
    
    if ~isempty(tableCompare.tblCHOMP)
        successCHOMP = successCHOMP + 1;
        [pl,avg] = reportTask(tableCompare.tblCHOMP);
        plCHOMP = plCHOMP + pl;
        avgCHOMP = avgCHOMP + avg;
    end

    if ~isempty(tableCompare.tblRRT)
        successRRT = successRRT + 1;
        [pl,avg] = reportTask(tableCompare.tblRRT);
        plRRT = plRRT + pl;
        avgRRT = avgRRT + avg;
    end
end
plLCQP = plLCQP / successLCQP;
avgLCQP = avgLCQP / successLCQP;
plCHOMP = plCHOMP / successCHOMP;
avgCHOMP = avgCHOMP / successCHOMP;
plRRT = plRRT / successRRT;
avgRRT = avgRRT / successRRT;
disp("Scenario 4: success "+successLCQP+"("+subsuccessLCQP+") "+successCHOMP+" "+successRRT+" "+numel(listCellscene1))
disp("Scenario 4: pathLength "+plLCQP+" "+plCHOMP+" "+plRRT)
disp("Scenario 4: avgJointMovement "+avgLCQP+" "+avgCHOMP+" "+avgRRT)

%%%%%%%%%%%%%%%%%%%%%%%%%% S5
successLCQP = 0;
subsuccessLCQP = 0;
successCHOMP = 0;
successRRT = 0;
plLCQP = 0;
plCHOMP = 0; 
plRRT = 0;
avgLCQP = 0;
avgCHOMP = 0;
avgRRT = 0;
worldObjects = {};
load("./matlab/data/scene5meta.mat");
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
for iCell = 1:numel(listCellscene5)
    load("./matlab/data/"+listCellscene5{iCell});
    if norm(tableCompare.goal - tableCompare.tblLCQP.x{end}') <= 0.01
        qCell = tableCompare.tblLCQP.q;
        isCollide = false;
        for iq = 1:numel(qCell)
            [isCollide,~,~] = checkCollision(rbt,qCell{iq},worldObjects,SkippedSelfCollisions="parent");
            if isCollide
                subsuccessLCQP = subsuccessLCQP + 1;
                break;
            end
        end
        if ~isCollide
            successLCQP = successLCQP + 1;
            [pl,avg] = reportTask(tableCompare.tblLCQP);
            plLCQP = plLCQP + pl;
            avgLCQP = avgLCQP + avg;
        end
    end
    
    if ~isempty(tableCompare.tblCHOMP)
        successCHOMP = successCHOMP + 1;
        [pl,avg] = reportTask(tableCompare.tblCHOMP);
        plCHOMP = plCHOMP + pl;
        avgCHOMP = avgCHOMP + avg;
    end

    if ~isempty(tableCompare.tblRRT)
        successRRT = successRRT + 1;
        [pl,avg] = reportTask(tableCompare.tblRRT);
        plRRT = plRRT + pl;
        avgRRT = avgRRT + avg;
    end
end
plLCQP = plLCQP / successLCQP;
avgLCQP = avgLCQP / successLCQP;
plCHOMP = plCHOMP / successCHOMP;
avgCHOMP = avgCHOMP / successCHOMP;
plRRT = plRRT / successRRT;
avgRRT = avgRRT / successRRT;
disp("Scenario 5: success "+successLCQP+"("+subsuccessLCQP+") "+successCHOMP+" "+successRRT+" "+numel(listCellscene1))
disp("Scenario 5: pathLength "+plLCQP+" "+plCHOMP+" "+plRRT)
disp("Scenario 5: avgJointMovement "+avgLCQP+" "+avgCHOMP+" "+avgRRT)