classdef TaskCHOMP < ITask
    %TASKCHOMP 
    
    properties
        rbt

        manipCHOMP

        t

        dt

        start

        goal
    end
    
    methods
        function obj = TaskCHOMP(t,dt)
            obj.rbt = loadrobot("frankaEmikaPanda","DataFormat","column");
            obj.rbt.removeBody("panda_hand");

            obj.t = t;
            obj.dt = dt;
        end
    end

    methods     
        function init(obj,positionList,radiusList,startConfig,endGoal)
            obj.manipCHOMP = manipulatorCHOMP(obj.rbt);
            obj.manipCHOMP.SphericalObstacles = [radiusList,positionList]';
            obj.manipCHOMP.SmoothnessOptions = chompSmoothnessOptions(SmoothnessCostWeight=1e-3);
            obj.manipCHOMP.CollisionOptions = chompCollisionOptions(CollisionCostWeight=10);
            obj.manipCHOMP.SolverOptions = chompSolverOptions(Verbosity="none",LearningRate=7.0);

            obj.start = startConfig;
            obj.goal = endGoal;
        end

        function run(obj)
            ht = tic;
            [wptsamples,tsamples] = obj.manipCHOMP.optimize([obj.start,obj.goal]',[0 obj.t+obj.dt],obj.dt);
            timeAll = toc(ht);
            obj.dataTable.time = tsamples';
            cellQ = cell(size(wptsamples,1),1);
            cellX = cell(size(wptsamples,1),1);
            for iCell = 1:size(wptsamples,1)
                cellQ{iCell} = wptsamples(iCell,:)';
                T = obj.rbt.getTransform(cellQ{iCell},"panda_link8","panda_link0");
                cellX{iCell} = T(1:3,4);
            end
            obj.dataTable.q = cellQ;
            obj.dataTable.x = cellX;
            obj.dataTable.iter = repelem(timeAll/numel(tsamples),numel(tsamples))';
        end
    end
end

