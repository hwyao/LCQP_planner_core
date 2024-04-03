classdef TaskLCPath < ITask
    %TASKLCPATH 
    
    properties
        dqVrep

        robotModel IRobotModel = RobotModelFrankaBar.empty

        controller IController = ControllerSimple.empty

        positionList(:,3) double

        start(7,1) double

        goal(3,1) double
    end
    
    methods
        function obj = TaskLCPath()
            obj.dqVrep = DQ_VrepInterface;
            obj.dqVrep.disconnect_all();
            obj.dqVrep.connect('127.0.0.1',19997);

            obj.robotModel = RobotModelFrankaBar("FrankaFix",obj.dqVrep);
        end
    end

    methods
        function init(obj,positionList,radiusList,startConfig,endGoal)
            obstacleCell = cell(1,size(positionList,1));
            for iObs = 1:size(positionList,1)
                iObstacleClass = ObstacleSphere("/Sphere["+(iObs-1)+"]",true,obj.dqVrep);
                iObstacleClass.radius = radiusList(iObs);
                obstacleCell{iObs} = iObstacleClass;
            end
            obj.positionList = positionList;
            obj.start = startConfig;
            obj.goal = endGoal;
            obj.controller = ControllerLCPath(obj.robotModel,obstacleCell,obj.goal);
            obj.controller.z = zeros(11, 1);
            varTypes = ["double","double","double","double"];
            varNames = ["time","q","x","iter"];
            obj.dataTable = table('Size',[0,4],'VariableNames',varNames,'VariableTypes',varTypes);
        end

        function run(obj)
            obj.controller.startSimulation();

            t = 0;
            
            while true
                % record the data
                newCell = {t,obj.controller.q,obj.controller.robotModel.fkm(obj.controller.q).translation.vec3(),obj.controller.tick};
                obj.dataTable = [obj.dataTable;newCell];

                if obj.controller.checkEnd() == true
                    break
                end
                
                % push the controller to next step
                obj.controller.nextStep(obj.positionList);
                t = t + obj.controller.dt;
            end
            
            obj.controller.stopSimulation();
        end
    end
end

