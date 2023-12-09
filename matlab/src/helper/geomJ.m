%
% Complement to get geometric jacobian
% Inputs: dq_kinematics object and joint configuration
% Output: 6xn geometric jacobian 
%
function [J] = geomJ(kine,q)
%
C8 = diag([-1 ones(1,3) -1 ones(1,3)]');
C4m = -C8(1:4,1:4);  
CJ4_2_J3= [zeros(3,1) eye(3)];
    

    Jacob = kine.pose_jacobian(q);
%     J = kuka.jacobian(q);
    xm = kine.fkm(q);
    J(1:3,:) = CJ4_2_J3*2*haminus4(xm.P')*Jacob(1:4,:); 
    J(4:6,:) = CJ4_2_J3*2*( hamiplus4(xm.D)*C4m*Jacob(1:4,:) +  haminus4(xm.P')*Jacob(5:8,:)  );
    
    
end