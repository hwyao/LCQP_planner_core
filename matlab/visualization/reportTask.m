function [pathLength,averageJointMovement] = reportTask(tbl)
    % path length
    pathMatrix = [tbl.x{:}];
    pathMatrixDiff = [pathMatrix(:,1),pathMatrix] - [pathMatrix,pathMatrix(:,end)];
    pathMatrixDiff = sqrt(sum(pathMatrixDiff.^2,1));
    pathLength = sum(pathMatrixDiff,'all');

    jointMatrix = [tbl.q{:}];
    jointMatrixDiff = [jointMatrix(:,1),jointMatrix] - [jointMatrix,jointMatrix(:,end)];
    jointMatrixDiff = abs(jointMatrixDiff);
    averageJointMovement = sum(jointMatrixDiff,2) / size(jointMatrixDiff,2);
    averageJointMovement = sum(averageJointMovement,"all")/7;
end

