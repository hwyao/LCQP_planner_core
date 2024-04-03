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

%%%%%%%%%%%%%%%%%%%%%%% S1

load("./matlab/data/scene1meta.mat");
load("./matlab/data/"+listCellscene1{1});
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
ax = show(rbt,tableCompare.configStart);
ax.NextPlot = "add";
%show(rbt,tableCompare.configGoal);
goal = tableCompare.goal;
scatter3(ax,goal(1),goal(2),goal(3),'red','filled');
show(worldObjects{1});
show(worldObjects{2});
show(worldObjects{3});
xlim(ax,[-0.6,0.6]);
ylim(ax,[-0.6,0.6]);
zlim(ax,[-0.2,0.8]);
view(ax,[96.409663990273955,20.829243070041034]);

print(gcf,'-vector','-dsvg','./matlab/fig/scene1');

%%%%%%%%%%%%%%%%%%%%%%%% S2
clf

load("./matlab/data/scene2meta.mat");
load("./matlab/data/"+listCellscene2{1});
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
ax = show(rbt,tableCompare.configStart);
ax.NextPlot = "add";
%show(rbt,tableCompare.configGoal);
goal = tableCompare.goal;
scatter3(ax,goal(1),goal(2),goal(3),'red','filled');
show(worldObjects{1});
show(worldObjects{2});
show(worldObjects{3});
xlim(ax,[-0.6,0.6]);
ylim(ax,[-0.6,0.6]);
zlim(ax,[-0.2,0.8]);
view(ax,[96.409663990273955,20.829243070041034]);

print(gcf,'-vector','-dsvg','./matlab/fig/scene2');

%%%%%%%%%%%%%%%%%%%%%%%% S3
clf

load("./matlab/data/scene3meta.mat");
load("./matlab/data/"+listCellscene3{1});
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
ax = show(rbt,tableCompare.configStart);
ax.NextPlot = "add";
%show(rbt,tableCompare.configGoal);
goal = tableCompare.goal;
scatter3(ax,goal(1),goal(2),goal(3),'red','filled');
show(worldObjects{1});
show(worldObjects{2});
show(worldObjects{3});
xlim(ax,[-0.6,0.6]);
ylim(ax,[-0.6,0.6]);
zlim(ax,[-0.2,1]);
view(ax,[96.409663990273955,20.829243070041034]);

print(gcf,'-vector','-dsvg','./matlab/fig/scene3');

%%%%%%%%%%%%%%%%%%%%%%%% S4
clf

load("./matlab/data/scene4meta.mat");
load("./matlab/data/"+listCellscene4{1});
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
ax = show(rbt,tableCompare.configStart);
ax.NextPlot = "add";
%show(rbt,tableCompare.configGoal);
goal = tableCompare.goal;
scatter3(ax,goal(1),goal(2),goal(3),'red','filled');
show(worldObjects{1});
show(worldObjects{2});
show(worldObjects{3});
xlim(ax,[-0.6,0.6]);
ylim(ax,[-0.6,0.6]);
zlim(ax,[-0.2,0.8]);
view(ax,[96.409663990273955,20.829243070041034]);

print(gcf,'-vector','-dsvg','./matlab/fig/scene4');

%%%%%%%%%%%%%%%%%%%%%%%% S5
clf

load("./matlab/data/scene5meta.mat");
load("./matlab/data/"+listCellscene5{1});
for iObs = 1:size(positionList,1)
    worldObjects{iObs} = collisionSphere(radiusList(iObs)); %#ok<*SAGROW>
    worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
end
ax = show(rbt,tableCompare.configStart);
ax.NextPlot = "add";
%show(rbt,tableCompare.configGoal);
goal = tableCompare.goal;
scatter3(ax,goal(1),goal(2),goal(3),'red','filled');
show(worldObjects{1});
show(worldObjects{2});
show(worldObjects{3});
xlim(ax,[-0.6,0.6]);
ylim(ax,[-0.6,0.6]);
zlim(ax,[-0.2,0.8]);
view(ax,[96.409663990273955,20.829243070041034]);

print(gcf,'-vector','-dsvg','./matlab/fig/scene5');