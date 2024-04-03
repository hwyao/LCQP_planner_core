function [isValid,configstart,goal,configGoal,worldObjects] = generateStartGoal(positionList,radiusList,options)
%GENERATESTARTGOAL 
    arguments
        positionList
        radiusList
        options.maxTrialPose = 15
        options.maxTrialIK = 6
        options.xLim = [-0.1,0.5]
        options.yLim = [-0.6,0.6]
        options.zLim = [0.2,0.8]
        options.distMin = 0.3
    end

    isValid = false;
    configstart = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]';
    goal = [0.3074,0.0006,0.6167];
    xLim = options.xLim;
    yLim = options.yLim;
    zLim = options.zLim;
    
    rbt = loadrobot("frankaEmikaPanda","DataFormat","column");
    rbt.removeBody("panda_hand");
    ik = inverseKinematics("RigidBodyTree",rbt);
    gik = generalizedInverseKinematics('RigidBodyTree',rbt,'ConstraintInputs',{'position'});
    posTgt = constraintPositionTarget("panda_link8");
    worldObjects = cell(size(positionList,1),1);
    jointConst = constraintJointBounds(rbt).Bounds;
    jointMin = jointConst(:,1);
    jointMax = jointConst(:,2);
    
    for iObs = 1:size(positionList,1)
        worldObjects{iObs} = collisionSphere(radiusList(iObs)); 
        worldObjects{iObs}.Pose(1:3,4) = positionList(iObs,:)'; 
    end
    

    for iTrial = 1:options.maxTrialPose
        while true
            start = [rand()*(xLim(2)-xLim(1))+xLim(1), rand()*(yLim(2)-yLim(1))+yLim(1), rand()*(zLim(2)-zLim(1))+zLim(1)];
            goal = [rand()*(xLim(2)-xLim(1))+xLim(1), rand()*(yLim(2)-yLim(1))+yLim(1), rand()*(zLim(2)-zLim(1))+zLim(1)];
            dist = sqrt(sum((start-goal).^2,'all'));
            if ~isInCollision(positionList,radiusList,start) && ~isInCollision(positionList,radiusList,goal) && dist > options.distMin
                break
            end
        end

        for iIK = 1:options.maxTrialIK
            u = rand(); v = rand(); w = rand();
            h = [sqrt(1 - u) * sin(2*pi*v), ...
                sqrt(1 - u) * cos(2*pi*v), ...
                sqrt(u) * sin(2*pi*w), ...
                sqrt(u) * cos(2*pi*w)];
            
            T_start = eye(4);
            T_start(1:3,1:3) = quat2rotm(h);
            T_start(1:3,4) = start';

            [configstart,~] = ik('panda_link8',T_start,[0.25 0.25 0.25 1 1 1],randomConfiguration(rbt));
            [isColliding,~,~] = checkCollision(rbt,configstart,worldObjects,SkippedSelfCollisions="parent");
            isOutLimit = any(configstart<jointMin,'all') || any(configstart>jointMax,'all');
            if isColliding(1) || isColliding(2) || isOutLimit
                continue
            end
            
            posTgt.TargetPosition = goal;
            [configGoal,~] = gik(configstart,posTgt);
            [isColliding,~,~] = checkCollision(rbt,configGoal,worldObjects,SkippedSelfCollisions="parent");
            isOutLimit = any(configGoal<jointMin,'all') || any(configGoal>jointMax,'all');
            if ~isColliding(1) && ~isColliding(2) &&~isOutLimit
                isValid = true;
                break
            end
        end
        if isValid
            break
        end
    end
end

function isCollide = isInCollision(positionList,radiusList,position)
    isCollide = false;
    for iObs = 1:size(positionList,1)
        pos = positionList(iObs,:);
        dist = pos - position;
        dist = sqrt(sum(dist.^2,'all'));
        if dist <= radiusList(iObs)
            isCollide = true;
            break
        end
    end
end

