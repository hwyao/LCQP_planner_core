classdef ITask < handle
    %ITASK The abstract class for task planning, prepared for code
    %experiment
    
    properties
        dataTable table
    end
    
    methods
        init(obj,positionList,radiusList,startConfig,endGoal)

        run(obj)
    end
end

