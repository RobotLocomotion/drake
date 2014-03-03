classdef FrictionConeWrenchConstraint < ContactWrenchConstraint
  % constrain the friction force to be within a friction cone.
  % @param body       -- The index of contact body on the robot
  % @param body_name  -- The name of the body.
  % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
  % of the contact point on the body frame.
  % @param num_pts    -- The number of contact points
  % @param FC_mu      -- A 1 x num_pts double vector, FC_mu(i) is the friction coefficient
  % of the friction cone at contact point body_pts(:,i)
  % @param FC_axis    -- A 3 x num_pts double matrix, FC_axis(:,i) is the axis of the
  % friction cone at the contact point body_pts(:,i). FC_axis(:,i) is in the world frame
  properties(SetAccess = protected)
    body
    body_name
    body_pts
    num_pts
    FC_mu
    FC_axis
  end
  
  properties(SetAccess = protected, GetAccess = protected)
    FC_lb;
    FC_ub;
  end
  
  methods
    function obj = FrictionConeWrenchConstraint(robot,body,body_pts,FC_mu,FC_axis,tspan)
      % @param body       -- The index of contact body on the robot
      % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
      % of the contact point on the body frame.
      % @param FC_mu      -- A 1 x num_pts double vector, FC_mu(i) is the friction coefficient
      % of the friction cone at contact point body_pts(:,i)
      % @param FC_axis    -- A 3 x num_pts double matrix, FC_axis(:,i) is the axis of the
      % friction cone at the contact point body_pts(:,i). FC_axis(:,i) is in the world frame
      % @param tspan      -- A 1 x 2 double vector. The time span of the constraint being
      % active
      if(nargin<6)
        tspan = [-inf inf];
      end
      obj = obj@ContactWrenchConstraint(robot,tspan);
      body_size = size(body);
      if(~isnumeric(body) || length(body_size) ~= 2 || body_size(1) ~= 1 || body_size(2) ~= 1)
        error('Drake:FrictionConeWrenchConstraint: body should be a numeric scalar');
      end
      obj.body = body;
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      body_pts_size = size(body_pts);
      if(~isnumeric(body_pts) || length(body_pts_size) ~= 2 || body_pts_size(1) ~= 3)
        error('Drake:FrictionConeWrenchConstraint: body_pts should be 3 x num_pts double matrix');
      end
      obj.body_pts = body_pts;
      obj.num_pts = body_pts_size(2);
      mu_size = size(FC_mu);
      if(length(mu_size) == 2 && mu_size(1) == 1 && mu_size(2) == 1)
        FC_mu = FC_mu*ones(1,obj.num_pts);
        mu_size = size(FC_mu);
      end
      if(~isnumeric(FC_mu) || any(FC_mu<0) || length(mu_size) ~= 2 || mu_size(1) ~= 1 || mu_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrenchConstraint: FC_mu should be a non-negative 1 x num_pts array');
      end
      obj.FC_mu = FC_mu;
      axis_size = size(FC_axis);
      if(length(axis_size) == 2 && axis_size(1) == 3 && axis_size(2) == 1)
        FC_axis = bsxfun(@times,FC_axis,ones(1,obj.num_pts));
        axis_size = size(FC_axis);
      end
      if(~isnumeric(FC_axis) || length(axis_size) ~= 2 || axis_size(1) ~= 3 || axis_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrenchConstraint: FC_axis should be a 3 x num_pts array');
      end
      FC_axis_norm = sqrt(sum(FC_axis.^2,1));
      obj.FC_axis = FC_axis./(bsxfun(@times,FC_axis_norm,ones(3,1)));
      obj.type = RigidBodyConstraint.FrictionConeWrenchConstraintType;
      obj.FC_lb = reshape(ones(1,obj.num_pts)./sqrt(obj.FC_mu.^2+ones(1,obj.num_pts)),[],1);
      obj.FC_ub = ones(obj.num_pts,1);
      obj.num_constraint = obj.num_pts;
      obj.F_size = [3,obj.num_pts];
      obj.type = RigidBodyConstraint.FrictionConeWrenchConstraintType;
      obj.F_lb = -inf(obj.F_size);
      obj.F_ub = inf(obj.F_size);
    end
    
    function [c,dc_val] = evalSparse(obj,t,kinsol,F)
      % This function evaluates the constraint and its non-zero entries in the sparse
      % gradient matrix.
      % @param t       - A scalar, the time to evaluate friction cone constraint
      % @param kinsol  - kinematics tree returned from doKinematics function
      % @param F       - A 3 x num_pts double matrix. The contact forces
      % @retval c      - A num_pts x 1 double vector, the constraint value
      % @retval dc_val - A 3*obj.num_pts x 1 double vector. The nonzero entries of constraint gradient w.r.t F
      if(obj.isTimeValid(t))
        valid_F = checkForceSize(obj,F);
        if(~valid_F)
          error('Drake:FrictionConeWrenchConstraint:friction force should be 3 x n_pts matrix');
        end
        F_norm = sqrt(sum(F.^2,1));
        c = reshape(sum(F.*obj.FC_axis,1)./F_norm,[],1);
        dc_entry = (obj.FC_axis'.*bsxfun(@times,(F_norm').^2,ones(1,3))-...
          bsxfun(@times,sum(F'.*obj.FC_axis',2),ones(1,3)).*F')./bsxfun(@times,(F_norm').^3,ones(1,3));
        dc_val = reshape(dc_entry',[],1);
      else
        c = [];
        dc_val = [];
      end
    end
    
    function [iCfun,jCvar,nnz] = evalSparseStructure(obj,t)
      % This function returns the sparsity structure of the constraint gradient.
      % sparse(iCfun,jCvar,dc,m,n,nnz) is the actual gradient matrix
      % @retval iCfun   -- A num_pts x 1 double vector. The row index of the nonzero entries
      % @retval jCvar   -- A num_pts x 1 double vector. The column index of the nonzero entries
      % @retval nnz     -- A scalar. The maximum non-zero entries in the sparse matrix.
      if(obj.isTimeValid(t))
        nq = obj.robot.getNumDOF;
        iCfun = reshape(bsxfun(@times,ones(3,1),(1:obj.num_pts)),[],1);
        jCvar = nq+(1:3*obj.num_pts)';
        nnz = 3*obj.num_pts;
      else
        iCfun = [];
        jCvar = [];
        nnz = 0;
      end
    end
       
    function [lb,ub] = bounds(obj,t)
      % The bounds of the friction cone constraint
      % @param t    -- A double scalar, the time to evaluate the friction cone constraint
      % @retval lb  -- A num_pts x 1 double vector. The lower bound of the constraint
      % @retval ub  -- A num_pts x 1 double vector. The upper bound of the constraint
      if(obj.isTimeValid(t))
        lb = obj.FC_lb;
        ub = obj.FC_ub;
      else
        lb = [];
        ub = [];
      end
    end
    
    function [tau,dtau] = torque(obj,t,kinsol,F)
      % Compute the total torque from contact position and contact force, together with
      % its gradient
      % @param F     -- a 3 x num_pts double matrix. F(:,i) is the contact force at
      % i'th contact point
      % @retval tau    -- a 3 x 1 double vector, the total torque around origin.
      % @retval dtau   -- a 3 x (nq+3*obj.num_pts) double matrix. The gradient of tau
      % w.r.t q and F
      if(obj.isTimeValid(t))
        valid_F = checkForceSize(obj,F);
        if(~valid_F)
          error('Drake:FrictionConeWrenchConstraint:friction force should be 3 x n_pts matrix');
        end
        nq = obj.robot.getNumDOF();
        [body_pos,dbody_pos] = forwardKin(obj.robot,kinsol,obj.body,obj.body_pts,0);
        tau = sum(cross(body_pos,F,1),2);
        dtau = zeros(3,nq+3*obj.num_pts);
        for i = 1:obj.num_pts
          dtau_tmp = dcross(body_pos(:,i),F(:,i));
          dtau(:,1:nq) = dtau(:,1:nq)+dtau_tmp(:,1:3)*dbody_pos((i-1)*3+(1:3),:);
          dtau(:,nq+(i-1)*3+(1:3)) = dtau_tmp(:,4:6);
        end
      else
        tau = [];
        dtau = [];
      end
    end
    
    
    function A = force(obj,t)
      % Compute total force from all the contact force parameters F. The total force is a
      % linear transformation from F. total_force = A*F(:)
      % @param t    -- A double scalar. The time to evaluate force
      % @param A    -- A 3 x 3*obj.num_pts double matrix.
      if(obj.isTimeValid(t))
        A = sparse(reshape(bsxfun(@times,(1:3)',ones(1,obj.num_pts)),[],1),...
          (1:3*obj.num_pts)',ones(3*obj.num_pts,1),3,3*obj.num_pts,3*obj.num_pts);
      else
        A = sparse(0,0);
      end
    end
    
    function name_str = name(obj,t)
      % Returns the name of each constraint in eval function
      % @param t         -- A double scalar, the time to evaluate constraint.
      % @retval name_str -- A obj.num_constraint x 1 cell. name_str{i} is the i'th
      % constraint name
      if(obj.isTimeValid(t))
        if(isempty(t))
          time_str = '';
        else
          time_str = sprintf('at time %5.2f',t);
        end
        name_str = cell(obj.num_constraint,1);
        for i = 1:obj.num_constraint
          name_str{i} = sprintf('Friction cone constraint for pts(:,%d) on %s %s',i,obj.body_name,time_str); 
        end
      else
        name_str = {};
      end
    end
    
    function [pos,J] = contactPosition(obj,t,kinsol)
      % Returns the contact positions and its gradient w.r.t q
      % @param t        -- A double scalar, the time to evaluate the contact position
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A matrix with 3 rows. pos(:,i) is the i'th contact position
      % @retval J       -- A matrix of size prod(size(pos)) x nq. The gradient of pos
      % w.r.t q
      if(obj.isTimeValid(t))
        [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.body_pts,0);
      else
        pos = [];
        J = [];
      end
    end
  end
  
end