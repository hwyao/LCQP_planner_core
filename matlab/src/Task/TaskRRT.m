classdef TaskRRT < ITask
    %TASKRRT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rbt

        manipRRT

        t

        dt

        start

        goal
    end
    
    methods
        function obj = TaskRRT(t,dt)
            obj.rbt = loadrobot("frankaEmikaPanda","DataFormat","column");
            obj.rbt.removeBody("panda_hand");

            obj.t = t;
            obj.dt = dt;
        end
    end

    methods     
        function init(obj,positionList,radiusList,startConfig,endGoal)
            worldObjects = cell(size(positionList,1),1);
            for iObs = 1:size(positionList,1)
                worldObjects{iObs} = collisionSphere(radiusList(iObs)); 
                worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
            end

            obj.manipRRT = manipulatorRRT(obj.rbt,worldObjects);
            obj.manipRRT.SkippedSelfCollisions = "parent";

            obj.start = startConfig;
            obj.goal = endGoal;
        end

        function run(obj)
            numSamples = round(obj.t / obj.dt) + 1;
            ht = tic;
            path = obj.manipRRT.plan(obj.start',obj.goal');
            [q,~,~,~,~] = trapveltraj(path',numSamples);
            q = q';
            timeAll = toc(ht);

            obj.dataTable.time = linspace(0,obj.t,numSamples)';
            cellQ = cell(size(q,1),1);
            cellX = cell(size(q,1),1);
            for iCell = 1:size(q,1)
                cellQ{iCell} = q(iCell,:)';
                T = obj.rbt.getTransform(cellQ{iCell},"panda_link8","panda_link0");
                cellX{iCell} = T(1:3,4);
            end
            obj.dataTable.q = cellQ;
            obj.dataTable.x = cellX;
            obj.dataTable.iter = repelem(timeAll/numel(obj.dataTable.time),numel(obj.dataTable.time))';
        end
    end
end

