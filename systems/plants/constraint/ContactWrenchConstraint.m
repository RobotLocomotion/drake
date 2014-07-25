classdef ContactWrenchConstraint < RigidBodyConstraint
  % constrain the contact forces
  % @param num_constraint   - A scalar. The number of constraints. 
  % @param F_size       - A 1 x 2 matrix. The size of the force parameter matrix.
  % @param F_lb         - A double matrix of size F_size. The lower bound on the
  % force parameters
  % @param F_ub         - A double matrix of size F_size. The upper bound on the
  % force parameters
  properties(SetAccess = protected)
    body % The index of contact body on the robot
    body_name % The name of the body.
    body_pts % A 3 x num_pts double matrix, each column represents the coordinate
             % of the contact point on the body frame.
    num_pts % The number of contact points
    num_constraint; % A scalar. The number of constraints. 
    pt_num_F; % An integer. The number of force parameters for one contact point
    F_lb; % A double matrix of size F_size. The lower bound on the
          % force parameters
    F_ub; % A double matrix of size F_size. The upper bound on the
          % force parameters
  end
  
  methods
    function obj = ContactWrenchConstraint(robot,body,body_pts,tspan)
      % @param robot    A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
      % object
      % @param body       -- The index of contact body on the robot
      % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
      % of the contact point on the body frame.
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.ContactWrenchConstraintCategory,robot,tspan);
      body_size = size(body);
      if(~isnumeric(body) || length(body_size) ~= 2 || body_size(1) ~= 1 || body_size(2) ~= 1)
        error('Drake:ContactWrenchConstraint: body should be a numeric scalar');
      end
      obj.body = body;
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      body_pts_size = size(body_pts);
      if(~isnumeric(body_pts) || length(body_pts_size) ~= 2 || body_pts_size(1) ~= 3)
        error('Drake:ContactWrenchConstraint: body_pts should be 3 x num_pts double matrix');
      end
      obj.body_pts = body_pts;
      obj.num_pts = body_pts_size(2);
    end
    
    function tspan = getTspan(obj)
      tspan = obj.tspan;
    end
    
    function flag = isTimeValid(obj,t)
      if(isempty(t))
        flag = true;
      else
        if(t>=obj.tspan(1)&&t<=obj.tspan(end))
          flag = true;
        else
          flag = false;
        end
      end
    end
    
    function n = getNumConstraint(obj,t)
      if(obj.isTimeValid(t))
        n = obj.num_constraint;
      else
        n = 0;
      end
    end
    
    function [w,dw] = wrenchSum(obj,kinsol,F)
      % computes the total wrench and its gradient w.r.t q and F
      % @param kinsol -- The kinematics tree, returned from doKinematics
      % @param F      -- A matrix of size obj.pt_num_F x obj.num_pts. The force parameter
      % @retval w     -- A 6 x 1 double vector. The total wrench 
      % @retval dw     -- A 6 x (nq+obj.pt_F_size*obj.num_pts matrix. The gradient of w w.r.t q
      % and F
      A = obj.forceSum();
      [tau,dtau] = obj.torqueSum(kinsol,F);
      w = [A*F(:);tau];
      dw = [zeros(3,obj.robot.getNumDOF()) A;dtau];
    end
    
    function [c,dc] = eval(obj,t,kinsol,F)
      % computes the constraint and its gradient.
      % @param t       -- A scalar. The time to evaluate the constraint
      % @param kinsol  -- kinematics tree returned from doKinematics function.
      % @param F       -- A double matrix of obj.pt_num_F*obj.num_pts. The contact forces parameters
      % @retval c      -- A obj.num_constraint x 1 vector, the constraint value.
      % @retval dc     -- A obj.num_constraint x obj.pt_num_F*obj.num_pts matrix. The gradient of c w.r.t F.
      if(obj.isTimeValid(t))
        [c,dc_val] = obj.evalSparse(kinsol,F);
        [iCfun,jCvar,nnz] = obj.evalSparseStructure();
        m = obj.getNumConstraint(t);
        n = obj.robot.getNumDOF+obj.pt_num_F*obj.num_pts;
        dc = sparse(iCfun,jCvar,dc_val,m,n,nnz);
      else
        c = [];
        dc = [];
      end
    end
    
    function cnstr = generateConstraint(obj,t)
      % generate a FunctionHandleConstraint for 'eval' function, and a BoundingBoxConstraint
      % for F
      if(obj.isTimeValid(t))
        [lb,ub] = obj.bounds(t);
        cnstr = {FunctionHandleConstraint(lb,ub,obj.robot.getNumDOF+prod(obj.F_size),@(kinsol,F) obj.eval(t,kinsol,F)),...
          BoundingBoxConstraint(obj.F_lb(:),obj.F_ub(:))};
        [iCfun,jCvar] = obj.evalSparseStructure();
        cnstr{1} = cnstr{1}.setSparseStructure(iCfun,jCvar);  
      else
        cnstr = {};
      end
    end
  end
  
  methods(Access = protected)
    function flag = checkForceSize(obj,F)
       F_size_tmp = size(F);
       flag = isnumeric(F) && length(F_size_tmp) == 2 && F_size_tmp(1) == obj.pt_num_F && F_size_tmp(2) == obj.num_pts;
    end
  end
  
  methods(Abstract)
    [c,dc_val] = evalSparse(obj,kinsol,F);
      % This function evaluates the constraint and its non-zero entries in the sparse
      % gradient matrix.
      % @param kinsol  - kinematics tree returned from doKinematics function
      % @param F       - A double matrix of obj.pt_num_F*obj.num_pts. The contact forces parameter
      % @retval c      - A double column vector, the constraint value. The size of the
      % vector is obj.getNumConstraint(t) x 1
      % @retval dc_val - A double column vector. The nonzero entries of constraint gradient w.r.t F
    [iCfun,jCvar,nnz] = evalSparseStructure(obj);
      % This function returns the sparsity structure of the constraint gradient.
      % sparse(iCfun,jCvar,dc,m,n,nnz) is the actual gradient matrix
      % @retval iCfun   -- A num_pts x 1 double vector. The row index of the nonzero entries
      % @retval jCvar   -- A num_pts x 1 double vector. The column index of the nonzero entries
      % @retval nnz     -- A scalar. The maximum non-zero entries in the sparse matrix.
    [tau,dtau] = torqueSum(obj,kinsol,F);
      % Compute the total torque from contact position and contact force, together with
      % its gradient
      % @param F       -- a matrix. The contact force parameter 
      % @retval tau    -- a 3 x 1 double vector, the total torque around origin.
      % @retval dtau   -- a 3 x (nq+obj.pt_num_F*obj.num_pts) double matrix. The gradient of tau
    A = forceSum(obj);
      % Compute total force from all the contact force parameters F. The total force is a
      % linear (sparse) transformation from F. total_force = A*F(:)
      % @param F    -- A double matrix of obj.pt_num_F*obj.num_pts. F is the force paramter
      % @param A    -- A 3 x obj.pt_num_F*obj.num_pts double matrix.
    A = force(obj)
      % Compute the indivisual force from the force parameters F. The individual forces
      % are reshape(A*F,3,obj.num_pts)
      % @retval A   -- A (3*num_pts) x (obj.pt_num_F*obj.num_pts) double matrix
    [lb,ub] = bounds(obj,t);
      % The bounds of the constraint returned by eval function
      % @param t    -- A double scalar, the time to evaluate the friction cone constraint
      % @retval lb  -- A double column vector. The lower bound of the constraint
      % @retval ub  -- A double column vector. The upper bound of the constraint
    name_str = name(obj,t);
      % Returns the name of each constraint in eval function
      % @param t         -- A double scalar, the time to evaluate constraint.
      % @retval name_str -- A obj.num_constraint x 1 cell. name_str{i} is the i'th
      % constraint name
    [pos,J] = contactPosition(obj,kinsol)
      % Compute the contact position and its gradient w.r.t q
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A matrix with 3 rows. pos(:,i) is the i'th contact position
      % @retval J       -- A matrix of size prod(size(pos)) x nq. The gradient of pos
      % w.r.t q
  end
end
