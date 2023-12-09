% This file is code of LCQP_planner_core project:
%   This script is the unreleased version of the project only for internal 
%   circulation. Any modification, distribution, private or commercial use 
%   outside the repository cooperation without the consent of all contributors 
%   is strictly forbidden. 
%   
% Contributor: Haowen Yao 
function [contactDist, contactPtObs, contactPtRobot, ...
          contactNormal, contactTransJacobian, contactTransJacobianGeometric] = barSphereContact(model,obstacle,q,iLink)
%BARSPHERECONTACT the physical calculation on how bar-model robot and sphere robot take into
% contact.
    arguments
        model RobotModelFrankaBar
        obstacle ObstacleSphere
        q(:,1) double
        iLink(1,1) double
    end
    
    % link information
    linkStart = model.kinematic_extra.get_pose(q,iLink-1).translation.vec3;
    linkEnd = model.kinematic_extra.get_pose(q,iLink).translation.vec3;
    linkLength = norm(linkStart-linkEnd);

    % obstacle information
    obstacleCenter = obstacle.center.vec3;
    obstacleRadius = obstacle.radius;
    
    startEndVecNorm = (linkEnd-linkStart)/linkLength;
    startObstacleVector = obstacleCenter - linkStart;
    projectToLinkLength = dot(startEndVecNorm,startObstacleVector);
    projectDirection = sign(projectToLinkLength);
    
    if projectDirection == -1
        contactPtRobot = linkStart;
    elseif projectToLinkLength > linkLength
        contactPtRobot = linkEnd;
    else
        contactPtRobot = linkStart + projectToLinkLength * startEndVecNorm;
    end

    if (isequal(contactPtRobot,obstacleCenter)) 
        % the special case handling to prevent NAN. Not neccessarily correct.
        contactNormal = [0;0;0];
        contactPtObs = obstacleCenter;
    else
        contactNormal = (contactPtRobot - obstacleCenter) / norm(contactPtRobot - obstacleCenter);
        contactPtObs = obstacleCenter + obstacleRadius * contactNormal;
    end

    contactDist = norm(contactPtRobot - contactPtObs);

    x = model.kinematic_extra.get_pose(q,iLink,projectToLinkLength);
    J = model.kinematic_extra.get_pose_jacobian(q,iLink,projectToLinkLength);
    contactTransJacobian = 2*haminus4(x.P')*J(5:8,:)+2*hamiplus4(x.D)*DQ.C4*J(1:4,:);
    CJ4_2_J3= [zeros(3,1) eye(3)];
    % % % CJ4_2_J3*2*( hamiplus4(xm.D)*C4m*Jacob(1:4,:) +  haminus4(xm.P')*Jacob(5:8,:)  );
    contactTransJacobianGeometric = 2*CJ4_2_J3*(haminus4(x.P')*J(5:8,:) + hamiplus4(x.D)*DQ.C4*J(1:4,:));
end

