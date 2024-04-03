% global setting
clear
clc
nBatch = 80;
savePath = "./matlab/data/";

% scene 1
positionList = [0.4,0.3,0.5;
                0.13,-0.32,0.5;
                0.2,0.5,0.4];
radiusList = [0.05;0.1;0.05];

for iBatch = 1:nBatch
    disp("=======scene 1 batch "+iBatch+"=========");
    tableCompare = taskCompare(positionList,radiusList);
    save(savePath+"scene1-"+string(tableCompare.saveTime),"tableCompare");
end
save(savePath+"scene1meta","positionList","radiusList");

% scene 2
positionList = [-0.1,0.3,0.5;
                0.4,-0.1,0.6;
                0.3,0.1,0.3];
radiusList = [0.05;0.1;0.05];

for iBatch = 1:nBatch
    disp("=======scene 2 batch "+iBatch+"=========");
    tableCompare = taskCompare(positionList,radiusList);
    save(savePath+"scene2-"+string(tableCompare.saveTime),"tableCompare");
end
save(savePath+"scene2meta","positionList","radiusList");

% scene 3
positionList = [0.2,0.4,0.4;
                0.1,0.0,0.8;
                -0.2,0.4,0.4];
radiusList = [0.05;0.1;0.05];

for iBatch = 1:nBatch
    disp("=======scene 3 batch "+iBatch+"=========");
    tableCompare = taskCompare(positionList,radiusList);
    save(savePath+"scene3-"+string(tableCompare.saveTime),"tableCompare");
end
save(savePath+"scene3meta","positionList","radiusList");


% scene 4
positionList = [0.4,0.0,0.5;
                0.4,0.3,0.3;
                0.3,-0.3,0.4];
radiusList = [0.05;0.1;0.05];

for iBatch = 1:nBatch
    disp("=======scene 4 batch "+iBatch+"=========");
    tableCompare = taskCompare(positionList,radiusList);
    save(savePath+"scene4-"+string(tableCompare.saveTime),"tableCompare");
end
save(savePath+"scene4meta","positionList","radiusList");

% scene 5
positionList = [0.425,0.0,0.425;
                0.48,-0.06,0.55;
                0.145,0.53,0.475];
radiusList = [0.05;0.1;0.05];
for iBatch = 1:nBatch
    disp("=======scene 5 batch "+iBatch+"=========");
    tableCompare = taskCompare(positionList,radiusList);
    save(savePath+"scene5-"+string(tableCompare.saveTime),"tableCompare");
end
save(savePath+"scene5meta","positionList","radiusList");