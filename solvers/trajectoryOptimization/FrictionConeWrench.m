classdef FrictionConeWrench < RigidBodyContactWrench
  % constrain the friction force to be within a friction cone.
  properties(SetAccess = protected)
    normal_dir % A 3 x 1 vector. The normal direction
    FC_mu % A 1 x num_pts double vector, FC_mu(i) is the friction coefficient
          % of the friction cone at contact point body_pts(:,i)
    FC_axis % A 3 x num_pts double matrix, FC_axis(:,i) is the axis of the
            % friction cone at the contact point body_pts(:,i). FC_axis(:,i) is in the world frame
    force_normalize_factor   % A positive double scaler. The actual force is force_normalize_factor*force_parameter,
                             % where force parameters are the arguments being constrained
  end
  
  properties(Access = protected)
    cos_theta_square; % theta being the half angle of the cone.
  end
  
  methods
    function obj = FrictionConeWrench(robot,body,body_pts,FC_mu,FC_axis,force_normalize_factor)
      % @param robot    A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
      % object
      % @param body       -- The index of contact body on the robot
      % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
      % of the contact point on the body frame.
      % @param FC_mu      -- A 1 x num_pts double vector or a scalar, FC_mu(i) is the friction coefficient
      % of the friction cone at contact point body_pts(:,i). If FC_mu is a scalar, then
      % that friciton coefficient is used for every contact point
      % @param FC_axis    -- A 3 x num_pts double matrix or a 3 x 1 vector, FC_axis(:,i) is the axis of the
      % friction cone at the contact point body_pts(:,i). FC_axis(:,i) is in the world
      % frame. If FC_axis is a 3 x 1 vector, then that axis is used for every friction
      % cone
      % @param force_normalize_factor   -- A positive double scalar. The actual force is
      % force_normalize_factor*force_parameter, where force_parameters are the argument to
      % the constraint. Default value is robot_mass*g
      obj = obj@RigidBodyContactWrench(robot,body,body_pts);
      mu_size = size(FC_mu);
      if(length(mu_size) == 2 && mu_size(1) == 1 && mu_size(2) == 1)
        FC_mu = FC_mu*ones(1,obj.num_pts);
        mu_size = size(FC_mu);
      end
      if(~isnumeric(FC_mu) || any(FC_mu<0) || length(mu_size) ~= 2 || mu_size(1) ~= 1 || mu_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrench: FC_mu should be a non-negative 1 x num_pts array');
      end
      obj.FC_mu = FC_mu;
      axis_size = size(FC_axis);
      if(length(axis_size) == 2 && axis_size(1) == 3 && axis_size(2) == 1)
        FC_axis = bsxfun(@times,FC_axis,ones(1,obj.num_pts));
        axis_size = size(FC_axis);
      end
      if(~isnumeric(FC_axis) || length(axis_size) ~= 2 || axis_size(1) ~= 3 || axis_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrench: FC_axis should be a 3 x num_pts array');
      end
      FC_axis_norm = sqrt(sum(FC_axis.^2,1));
      obj.FC_axis = FC_axis./(bsxfun(@times,FC_axis_norm,ones(3,1)));
      obj.normal_dir = obj.FC_axis;
      g = 9.81;
      if(nargin<6)
        force_normalize_factor = obj.robot.getMass*g;
      else
        if(~isnumeric(force_normalize_factor) || numel(force_normalize_factor) ~= 1 || force_normalize_factor<=0)
          error('Drake:RigidBodyConeWrench: force_normalize_factor should be a positive scalar');
        end
      end
      obj.force_normalize_factor = force_normalize_factor;
      obj.cos_theta_square = 1./(obj.FC_mu.^2+ones(1,obj.num_pts));
      obj.num_wrench_constraint = obj.num_pts;
      obj.num_pt_F = 3;
      obj.F_lb = -inf(obj.num_pt_F,obj.num_pts);
      obj.F_ub = inf(obj.num_pt_F,obj.num_pts);
      obj.contact_force_type = RigidBodyContactWrench.FrictionConeType;
      obj.wrench_cnstr_lb = zeros(obj.num_pts,1);
      obj.wrench_cnstr_ub = inf(obj.num_pts,1);
      obj.wrench_cnstr_name = cell(obj.num_pts,1);
      for i = 1:obj.num_pts
        obj.wrench_cnstr_name{i} = sprintf('%s_pt%d_friction_cone',obj.body_name,i);
      end
      obj.wrench_iCfun = reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1);
      obj.wrench_jCvar = obj.robot.getNumPositions+(1:3*obj.num_pts)';
    end
    
    function [c,dc] = evalWrenchConstraint(obj,kinsol,F,slack)
      % This function evaluates the constraint and its non-zero entries in the sparse
      % gradient matrix.
      % @param t       - A scalar, the time to evaluate friction cone constraint
      % @param kinsol  - kinematics tree returned from doKinematics function
      % @param F       - A 3 x num_pts double matrix. The contact forces
      % @retval c      - A num_pts x 1 double vector, the constraint value
      % @retval dc     - A obj.num_pts x (3*obj.num_pts) double matrix. The gradient of c w.r.t F
      valid_F = checkForceSize(obj,F);
      if(~valid_F)
        error('Drake:RigidBodyFrictionCone:friction force should be 3 x n_pts matrix');
      end
      % The constraint is (F'*axis)^2>cos_theta^2*F_norm_square and F'*axis>0
      F_norm_square = sum(F.^2,1);
      F_times_axis = sum(F.*obj.FC_axis,1);
      c = reshape(F_times_axis.^2-obj.cos_theta_square.*F_norm_square,[],1);
      dc_val = reshape((bsxfun(@times,2*F_times_axis',ones(1,3)).*obj.FC_axis'-2*bsxfun(@times,obj.cos_theta_square',ones(1,3)).*F')',[],1);
      nq = obj.robot.getNumPositions;
      dc = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),nq+(1:3*obj.num_pts)',dc_val,obj.num_pts,nq+3*obj.num_pts);
    end
    
    function A = force(obj)
      % Compute the indivisual force from the force parameters F. The individual forces
      % are reshape(A*F,3,obj.num_pts)
      % @retval A    -- A (3*num_pts) x (obj.num_pt_F*obj.num_pts) double matrix
      A = obj.force_normalize_factor*speye(3*obj.num_pts);
    end
    
    function A = torque(obj)
      % Compute the torque at each contact point from the force parameter.
      % @retval A   -- A (3*num_pts) x (obj.num_pt_F*obj.num_pts) double matrix.
      % reshape(A*F(:),3,num_pts) are the torque at each contact point
      A = sparse(3*obj.num_pts,obj.num_pt_F*obj.num_pts);
    end
    
    function [pos,J] = contactPosition(obj,kinsol)
      % Returns the contact positions and its gradient w.r.t q
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A matrix with 3 rows. pos(:,i) is the i'th contact position
      % @retval J       -- A matrix of size prod(size(pos)) x nq. The gradient of pos
      % w.r.t q
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.body_pts,0);
    end
  end
  
  methods(Access = protected)
    function lincon = generateWrenchLincon(obj)
      lincon_mat = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:obj.num_pt_F*obj.num_pts)',obj.FC_axis(:),obj.num_pts,3*obj.num_pts);
      lincon = LinearConstraint(zeros(obj.num_pts,1),inf(obj.num_pts,1),lincon_mat);
      lincon_name = cell(obj.num_pts,1);
      for i = 1:obj.num_pts
        lincon_name{i} = sprintf('%s_pt%d_friction_cone',obj.body_name,i);
      end
      lincon = lincon.setName(lincon_name);
    end
  end
end