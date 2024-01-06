%FRANKAKINEMATIC calculate the franka kinematic information
% important methods:
%   get_pose()  get the forward kinematic
%   get_pose_jacobian() get the jacobian matrix (in DQ)
%   get_geometric_jacobian() get the jacobian matrix (geometric)
% The most improtant functionality is that this script could to get
% the jacobian of robot at any point.

% License:
%   This script is part of the project and is based on DQ Robotics, which
%   is licensed under the LGPL (Lesser General Public License). This file 
%   is used by other part of the code, have no strict connection to the 
%   rest of the code. As a result, this script alone inherits the LGPL 
%   license and takes precedence over the overall license of this project.
%
%   Note: As the early stage of the development, this file is unpublished 
%   and is only for internal circulation, which does not voilate the LGPL condition.
%   The script is currently under development and may be unstable.
%   Upon completion, stabilization and the publication of the repository, 
%   there are intentions to contribute back to the DQ Robotics repository
%   (if DQ Robotics accepts it), resolving the license inconsistencies.
%
% Author: Haowen Yao - haowen.yao@tum.de
classdef FrankaKinematic < handle
    properties %(SetAccess = immutable)
        % the DH information of the robot
        theta
        d
        a
        alpha
        n_links

        % the default EE shift of the robot
        EE

        % the length of each link
        link_length
        EE_length
    end

    properties
        % the temporary stored configuration
        q

        % the list of dq transformation from DH parameter 
        DH_DQ_full

        % the mark that DH_DQ_full is calculated stored and not evaluated 
        DH_DQ_full_storage_mark
    end
    
    methods
        function obj = FrankaKinematic(DH,EE)
            arguments
                DH(4,:) double
                EE(1,1) DQ
            end
            obj.n_links = size(DH,2);
            obj.theta = DH(1,:);
            obj.d = DH(2,:);
            obj.a = DH(3,:);
            obj.alpha = DH(4,:);

            obj.EE = EE;

            obj.link_length = sqrt(obj.d.^2+obj.a.^2);
            obj.EE_length = norm(EE.translation.vec3());
            obj.q = zeros(7,1);
            obj.DH_DQ_full = repelem(DQ(0),1,7);
            obj.DH_DQ_full_storage_mark = false(1,7);
        end

        function check_q_update(obj,q)
            % CHECK_Q_UPDATE check if the configuration q is updated.
            % If this q input is different than obj.q, all buffered data should
            % be marked as invalid in DH_DQ_full_storage_mark
            if ~isequal(q,obj.q)
                obj.DH_DQ_full_storage_mark = false(1,7);
            end
        end

        function dq = dh2dq_raw(~,theta,d,a,alpha)
            % DH2DQ_RAW calculate the rotation of each link according to
            % the input DH parameter.

            % standard
            % h(1) = cos((theta+obj.theta(ith))/2)*cos(alpha/2);
            % h(2) = cos((theta+obj.theta(ith))/2)*sin(alpha/2);
            % h(3) = sin((theta+obj.theta(ith))/2)*sin(alpha/2);
            % h(4) = sin((theta+obj.theta(ith))/2)*cos(alpha/2);
            % d2 = d/2;
            % a2 = a/2;
            % 
            % h(5) = -d2*h(4)-a2*h(2);
            % h(6) = -d2*h(3)+a2*h(1);
            % h(7) = d2*h(2)+a2*h(4);
            % h(8) = d2*h(1)-a2*h(3);
            
            % modified (for franka)
            h1 = cos(theta/2)*cos(alpha/2);
            h2 = cos(theta/2)*sin(alpha/2);
            h3 = sin(theta/2)*sin(alpha/2);
            h4 = sin(theta/2)*cos(alpha/2);
            h(1) = h1;
            h(2) = h2;
            h(3) = -h3;
            h(4) = h4;
            d2 = d/2;
            a2 = a/2;
            
            h(5) = -d2*h4-a2*h2;
            h(6) = -d2*h3+a2*h1;
            h(7) = -(d2*h2+a2*h4);
            h(8) = d2*h1-a2*h3;
            
            dq = DQ(h);
        end

        function [d_out,a_out] = interpolate_DH_param(obj,d,a,ith,length)
            % INTERPOLATE_DH_PARAM interpolate the DH parameter so that the
            % d and a can have be scaled down to have a link with specific length. 
            if length < 0
                length_real = 0;
            elseif length > obj.link_length(ith)
                length_real = obj.link_length(ith); 
            else
                length_real = length;
            end

            if length_real == 0
                d_out = 0;
                a_out = 0;
            else
                d_out = d * length_real / obj.link_length(ith);
                a_out = a * length_real / obj.link_length(ith);
            end
        end
   
        function dq = dh2dq(obj,theta,ith,length)
            % DH2DQ For a given link's modified DH parameters, calculate the correspondent dual
            % quaternion. The result is either from buffer or newly calculated.
            % Usage: 
            %   dq = dh2dq(theta,i)
            % Parameter: 
            %   theta: joint angle
            %   ith: link number, could be between [1,n_link]
            %   length(optional): the length of the link, could be be any value but if it 
            %   exceeds [0,link_length], it will be set to the limit value.
            % return the dual quaterion represention frame transformation.
            if nargin == 3 && obj.DH_DQ_full_storage_mark(ith)==true
                dq = obj.DH_DQ_full(ith);
            else
                theta_in = theta + obj.theta(ith);
                d_in = obj.d(ith);
                a_in = obj.a(ith);
                if nargin == 4
                    [d_in,a_in] = obj.interpolate_DH_param(d_in,a_in,ith,length);
                end
                alpha_in = obj.alpha(ith);
                dq = obj.dh2dq_raw(theta_in,d_in,a_in,alpha_in);
                if nargin == 3
                    obj.DH_DQ_full(ith) = dq;
                    obj.DH_DQ_full_storage_mark(ith) = true;
                end
            end            
        end

        function dq = interpolate_EE_shift(obj,length)
            % INTERPOLATE_EE_SHIFT interpolate the end-effector shifting. 
            if length < 0
                length_real = 0;
            elseif length > obj.EE_length
                length_real = obj.EE_length; 
            else
                length_real = length;
            end
            ee_rot = obj.EE.rotation;
            ee_trans = obj.EE.translation * length_real / obj.EE_length;
            dq = ee_rot + DQ.E * 0.5 * ee_rot * ee_trans;
        end

        function x = get_pose(obj,q,ith,length)
            % GET_POSE Get the pose of the robot with the on ith link with the corresponding 
            % link length.
            % Usage:
            %   get_pose(q)
            %   get_pose(q,ith)
            %   get_pose(q,ith,length)
            % Parameter:
            %   q: the joint configuration
            %   ith (optional): the link number. could be between [1,n_link+1],
            %      which that represent the 1-nth link or the EE shift. For
            %      default
            %   length (optional): the length of the link, could be be any
            %   value but if it exceeds [0,link_length], it will be set to the
            %   limit value.
            % return the position in dual quaterion.
            obj.check_q_update(q);
            
            x = DQ(1);
            if nargin == 2 % get_pose(q)
                for i=1:obj.n_links
                    x = x*obj.dh2dq(q(i),i);
                end
                x = x * obj.EE;

            elseif nargin == 3 % get_pose(q,ith)
                for i=1:ith
                    x = x*obj.dh2dq(q(i),i);
                end

            elseif nargin == 4 % get_pose(q,ith,length)
                for i=1:ith-1
                    x = x*obj.dh2dq(q(i),i);
                end
                if ith == obj.n_links + 1
                    x = x*obj.interpolate_EE_shift(length);
                else
                    x = x*obj.dh2dq(q(ith),ith,length);
                end
            else
                error("Wrong argument input length.")
            end
        end
        
        function J = get_pose_jacobian(obj,q,ith,length)
            % GET_POSE_JACOBIAN to get the Jacobian of the robot with the ith link with the
            % corresponding link length. 
            % Usage:
            %   get_pose_jacobian(q)
            %   get_pose_jacobian(q,ith)
            %   get_pose_jacobian(q,ith,length)
            % Parameter:
            %   q: the joint configuration
            %   ith (optional): the link number. could be between [1,n_link+1],
            %      which that represent the 1-nth link or the EE shift. For
            %      empty default we consider the EE.
            %   length (optional): the length of the link, could be be any
            %      value but if it exceeds [0,link_length], it will be set to the
            %      limit value. For empty default we consider the full length.
            % return the 8*ith matrix. ith maximum to link_length.
            obj.check_q_update(q);

            if nargin == 2
                x_EE = obj.get_pose(q,obj.n_links);
                n = obj.n_links;
            elseif nargin == 3
                x_EE = obj.get_pose(q,ith);
                n = ith;
            elseif nargin == 4
                if ith == obj.n_links+1
                    x_EE = obj.get_pose(q,7);
                else
                    x_EE = obj.get_pose(q,ith,length);
                end
                n = ith - 1;
            else
                error("Wrong argument input length.")
            end

            x = DQ(1);
            J = zeros(8,n);

            for i = 0 : n - 1 
                w = DQ([0,  0,  -sin(obj.alpha(i+1)),               cos(obj.alpha(i+1)), ...
                        0,  0,  -obj.a(i+1)*cos(obj.alpha(i+1)),    -obj.a(i+1)*sin(obj.alpha(i+1))] ); % modified DH
                z = 0.5*x*w*x';
                
                x = x*obj.dh2dq(q(i+1),i+1);
                j = z * x_EE;
                J(:,i+1) = vec8(j);
            end

            if nargin == 2
                J = haminus8(obj.EE)*J;
            elseif nargin == 3
                % do nothing
            elseif nargin == 4
                if ith == obj.n_links + 1
                    J = haminus8(obj.interpolate_EE_shift(length))*J;
                else
                    n = ith;
                    d_in = obj.d(n);
                    a_in = obj.a(n);
                    [~,a_in] = obj.interpolate_DH_param(d_in,a_in,ith,length);
                    w = DQ([0,  0,  -sin(obj.alpha(n)),         cos(obj.alpha(n)), ...
                            0,  0,  -a_in*cos(obj.alpha(n)),    -a_in*sin(obj.alpha(n))] );
                    z = 0.5*x*w*x';
                    j = z * x_EE;
                    J(:,n) = vec8(j);
                end
            end
        end

        function J = get_geometric_jacobian(obj,q)
            % GET_GEOMETRIC_JACOBIAN to get geometric Jacobian of the robot with specific configuration
            % Usage:
            %   get_pose_jacobian(q)
            C8 = diag([-1 ones(1,3) -1 ones(1,3)]');
            C4m = -C8(1:4,1:4);
            CJ4_2_J3= [zeros(3,1) eye(3)];

            Jacob = obj.get_pose_jacobian(q);
            xm = obj.get_pose(q);
            J(1:3,:) = CJ4_2_J3*2*haminus4(xm.P')*Jacob(1:4,:);
            J(4:6,:) = CJ4_2_J3*2*( hamiplus4(xm.D)*C4m*Jacob(1:4,:) +  haminus4(xm.P')*Jacob(5:8,:));
        end
    end
end

