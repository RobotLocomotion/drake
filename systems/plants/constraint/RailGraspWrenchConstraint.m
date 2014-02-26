classdef RailGraspWrenchConstraint < ContactWrenchConstraint
  % Constrain the force and torque at the contact point when the hand is grasping a rail. The force is subject
  % to magnitude constraint and friction cone constraint. The bounds on the torque is a
  % cylinder co-axial with the rail
  % @param body               -- The index of contact body
  % @param body_name          -- The name of the body
  % @param grasp_center_pt     -- A 3 x 1 double matrix, a point on the body frame that is
  % the center of the grasping
  % @param grasp_length  -- A non negative scalar. The length of the grasping region
  % projected onto the rail
  % @param rail_axis   -- A 3 x 1 vector, the axis of the rail
  % @param rail_radius -- A non-negative scalar, the radius of the rail
  % @param rail_fc_mu  -- A non-negative scalar, the friction coefficient on the rail
  % @param force_max    -- A positive scalar, the maximum maginitude the force
  properties(SetAccess = protected)
    body
    body_name
    grasp_center_pt
    grasp_length
    num_pts
    rail_axis
    rail_radius
    rail_fc_mu
    force_ub;
    force_max;
  end
  
  properties(SetAccess = protected, GetAccess = protected)
    % @param torque_bnd_height   -- A non-negative double scalar. The height of the
    % cylinder that bounds the torque
    % @param torque_bnd_radius   -- A non-negative double scalar. The radius of the
    % cylinder that bounds the torque
    torque_bnd_height
    torque_bnd_radius
  end
  
  methods
    function obj = RailGraspWrenchConstraint(robot,body,grasp_center_pt,grasp_length,rail_axis,rail_radius,rail_fc_mu,force_max,tspan)
      % cylinder co-axial with the rail
      % @param body               -- The index of contact body
      % @param grasp_center_pt     -- A 3 x 1 double matrix, a point on the body frame that is
      % the center of the grasping
      % @param grasp_length  -- A non negative scalar. The length of the grasping region
      % projected onto the rail
      % @param rail_axis   -- A 3 x 1 vector, the axis of the rail
      % @param rail_radius -- A non-negative scalar, the radius of the rail
      % @param rail_fc_mu  -- A non-negative scalar, the friction coefficient on the rail
      % @param force_max    -- A positive scalar, the maximum maginitude the force
      % @param tspan        -- A 1 x 2 double vector. The time span of the constraint
      % being active
      if(nargin<9)
        tspan = [-inf,inf];
      end
      obj = obj@ContactWrenchConstraint(robot,tspan);
      body_size = size(body);
      if(~isnumeric(body) || length(body_size) ~= 2 || body_size(1) ~= 1 || body_size(2) ~= 1)
        error('Drake:RailGraspWrenchConstraint: body should be a numeric scalar');
      end
      obj.body = body;
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      pt_size = size(grasp_center_pt);
      if(~isnumeric(grasp_center_pt) || pt_size(1) ~= 3 || pt_size(2) ~= 1)
        error('Drake:RailGraspWrenchConstraint: grasp_center_pt should be a 3 x 1 vector');
      end
      obj.grasp_center_pt = grasp_center_pt;
      obj.num_pts = 1;
      if(~isnumeric(grasp_length) || numel(grasp_length) ~= 1 || grasp_length<0)
        error('Drake:RailGraspWrenchConstraint: grasp_length should be a non-negative scalar');
      end
      obj.grasp_length = grasp_length;
      rail_axis_size = size(rail_axis);
      if(~isnumeric(rail_axis_size) || rail_axis_size(1) ~= 3 || rail_axis_size(2) ~= 1)
        error('Drake:RailGraspWrenchConstraint: rail_axis should be a 3 x 1 vector');
      end
      rail_axis_norm = norm(rail_axis);
      if(rail_axis_norm<1e-10)
        error('Drake:RailGraspWrenchConstraint: rail_axis should be a non-zero vector');
      end
      obj.rail_axis = rail_axis/rail_axis_norm;
      if(~isnumeric(rail_radius) || numel(rail_radius) ~= 1 || rail_radius<0)
        error('Drake:RailGraspWrenchConstraint: rail_radius should be a non-negative scalar');
      end
      obj.rail_radius = rail_radius;
      if(~isnumeric(rail_fc_mu) || numel(rail_fc_mu) ~= 1 || rail_fc_mu<0)
        error('Drake:RailGraspWrenchConstraint: rail_fc_mu should be a non-negative scalar');
      end
      obj.rail_fc_mu = rail_fc_mu;
      if(~isnumeric(force_max) || numel(force_max) ~= 1 || force_max <= 0)
        error('Drake:RailGraspWrenchConstraint: force_max should be a positive scalar');
      end
      obj.force_max = force_max;
      obj.torque_bnd_height = obj.force_max*obj.rail_radius;
      obj.torque_bnd_radius = obj.force_max*obj.grasp_length/2;
      obj.num_constraint = 4;
      obj.F_size = [6,1];
      obj.type = RigidBodyConstraint.RailGraspWrenchConstraintType;
      obj.F_lb = -[obj.force_max*ones(3,1);sqrt(obj.torque_bnd_height^2+obj.torque_bnd_radius^2)*ones(3,1)];
      obj.F_ub = [obj.force_max*ones(3,1);sqrt(obj.torque_bnd_height^2+obj.torque_bnd_radius^2)*ones(3,1)];
    end
    
    function [lb,ub] = bounds(obj,t)
      % The bounds of the friction cone constraint
      % @param t    -- A double scalar, the time to evaluate the friction cone constraint
      % @retval lb  -- A 4 x 1 double vector. The lower bound of the constraint
      % @retval ub  -- A 4 x 1 double vector. The upper bound of the constraint
      if(obj.isTimeValid(t))
        fc_bnd = obj.rail_fc_mu/sqrt(obj.rail_fc_mu^2+1);
        lb = [-fc_bnd;0;-obj.torque_bnd_height;0];
        ub = [fc_bnd;obj.force_max;obj.torque_bnd_height;obj.torque_bnd_radius];
      else
        lb = [];
        ub = [];
      end
    end
    function [c,dc_val] = evalSparse(obj,t,kinsol,F)
      % This function evaluates the friction cone constraint on the force,magnitude
      % constraint on the force, and cylinder constraint on the torque.
      % @param t       - A scalar, the time to evaluate friction cone constraint
      % @param kinsol  - kinematics tree returned from doKinematics function
      % @param F       - A 6 x 1 double vector. The contact forces and torque about the
      % contact point
      % @retval c      - A 4 x 1 double vector, the constraint value
      % @retval dc_val - A double column vector. The nonzero entries of constraint gradient w.r.t F 
      if(obj.isTimeValid(t))
        f_mag = norm(F(1:3));
        df_magdforce = F(1:3)'/f_mag;
        fc_cnst = F(1:3)'*obj.rail_axis/f_mag;
        dfc_cnstdforce = (obj.rail_axis'*f_mag-F(1:3)'*obj.rail_axis*df_magdforce)/f_mag^2;
        tau = F(4:6);
        tau_axis = tau'*obj.rail_axis;
        dtau_axisdtau = obj.rail_axis';
        tau_radius = norm(tau-tau_axis*obj.rail_axis);
        dtau_radiusdtau = (tau-tau_axis*obj.rail_axis)'*(eye(3)-obj.rail_axis*dtau_axisdtau)/tau_radius;
        c = [fc_cnst;f_mag;tau_axis;tau_radius];
        dc_val = [dfc_cnstdforce(:);df_magdforce(:);dtau_axisdtau(:);dtau_radiusdtau(:)];
      else
        c = [];
        dc_val = [];
      end
    end
    
    function [iCfun,jCvar,nnz] = evalSparseStructure(obj,t)
      % This function returns the sparsity structure of the constraint gradient.
      % sparse(iCfun,jCvar,dc_val,m,n,nnz) is the actual gradient matrix
      % @retval iCfun   -- A num_pts x 1 double vector. The row index of the nonzero entries
      % @retval jCvar   -- A num_pts x 1 double vector. The column index of the nonzero entries
      % @retval nnz     -- A scalar. The maximum non-zero entries in the sparse matrix.
      if(obj.isTimeValid(t))
        nq = obj.robot.getNumDOF;
        iCfun = reshape(bsxfun(@times,ones(3,1),(1:4)),[],1);
        jCvar = nq+[1;2;3;1;2;3;4;5;6;4;5;6];
        nnz = 12;
      else
        iCfun = [];
        jCvar = [];
        nnz = 0;
      end
    end
    
    function [tau,dtau] = torque(obj,t,kinsol,F)
      % Compute the total torque from contact position and contact force, together with
      % its gradient
      % @param F       -- a 6 x 1 double vector. The contact force and torque at the
      % contact point
      % @retval tau    -- a 3 x 1 double vector, the total torque around origin.
      % @retval dtau   -- a 3 x (nq+6) double matrix. The gradient of tau
      if(obj.isTimeValid(t))
        valid_F = checkForceSize(obj,F);
        if(~valid_F)
          error('Drake:RailGraspWrenchConstraint:friction force should be 6 x 1 matrix');
        end
        [pos,dpos] = obj.robot.forwardKin(kinsol,obj.body,obj.grasp_center_pt,0);
        tau = cross(pos,F(1:3))+F(4:6);
        dtau1 = dcross(pos,F(1:3));
        dtau = [dtau1(:,1:3)*dpos dtau1(:,4:6) eye(3)];
      else
        tau = [];
        dtau = [];
      end
    end
    
    function A = force(obj,t)
      % Compute total force from all the contact force parameters F. The total force is a
      % linear transformation from F. total_force = A*F(:)
      % @param t    -- A double scalar. The time to evaluate force
      % @param A    -- A 3 x 6 double matrix.
      if(obj.isTimeValid(t))
        A = [speye(3) sparse(3,3)];
      else
        A = sparse(0,0);
      end
    end
    
    function name_str = name(obj,t)
      % Returns the name of each constraint in eval function
      % @param t         -- A double scalar, the time to evaluate constraint.
      % @retval name_str -- A 4 x 1 cell. name_str{i} is the i'th
      % constraint name
      if(obj.isTimeValid(t))
        if(isempty(t))
          time_str = '';
        else
          time_str = sprintf('at time %5.2f',t);
        end
        name_str = cell(obj.num_constraint,1);
        name_str{1} = sprintf('Rail grasp friction cone constraint %s',time_str);
        name_str{2} = sprintf('Rail grasp force magnitue constraint %s',time_str);
        name_str{3} = sprintf('Rail grasp torque in cylinder constraint %s',time_str);
        name_str{4} = sprintf('Rail grasp torque in cylinder constraint %s',time_str);
      else
        name_str = {};
      end
    end
    
    function [pos,J] = contactPosition(obj,t,kinsol)
      % Returns the contact positions and its gradient w.r.t q
      % @param t        -- A double scalar, the time to evaluate the contact position
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A 3 x 1 double vector. The position of the center grasp point
      % @retval J       -- A 3 x nq double matrix. The gradient of pos
      % w.r.t q
      if(obj.isTimeValid(t))
        [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.grasp_center_pt,0);
      else
        pos = [];
        J = [];
      end
    end
  end
end