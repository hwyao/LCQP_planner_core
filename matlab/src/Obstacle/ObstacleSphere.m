classdef ObstacleSphere < IObstacle
    %CLASSNAME Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = immutable)
        name

        isActive
    end
    
    properties 
        dqVrep = DQ_VrepInterface.empty;
    end

    properties
        center(1,1) DQ = DQ(1);

        radius(1,1) double
    end
    
    methods
        function obs = ObstacleSphere(name,isActive,dqVrep)
            obs.name = name;
            obs.isActive = isActive;
            obs.dqVrep = dqVrep;
        end
    end

    methods
        function updateStatus(obs,center)
            if obs.isActive == 0
                obs.center = obs.dqVrep.get_object_translation(obs.name);
            else
                obs.center = DQ(center);
                obs.dqVrep.set_object_translation(obs.name,obs.center);
            end
        end
    end
end
