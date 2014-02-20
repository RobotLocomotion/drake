classdef ContactWrenchConstraint < RigidBodyConstraint
  % constrain the contact forces
  % @param tspan   - A 1 x 2 vector. The time span of the constraint being active
  % @param robot   - A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
  % @param num_constraint   - A scalar. The number of constraints. 
  % @param mex_ptr          - A DrakeConstraintMexPointer
  % @param force_size       - A 1 x 2 matrix. The size of the force matrix.
  % @param force_lb         - A double matrix of size force_size. The lower bound on the
  % force
  % @param force_ub         - A double matrix of size force_size. The upper bound on the
  % force
  properties(SetAccess = protected)
    tspan % a 1x2 vector
    num_constraint;
    robot
    mex_ptr
    force_size;
    force_lb;
    force_ub
  end
  
  methods
    function obj = ContactWrenchConstraint(robot,tspan)
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.ContactWrenchConstraintCategory);
      if(nargin<2)
        tspan = [-inf,inf];
      end
      if(isempty(tspan))
        tspan = [-inf,inf];
      end
      if(tspan(1)>tspan(end))
        error('Drake:ContactWrenchConstraint:tspan(1) should be no larger than tspan(end)')
      end
      obj.tspan = [tspan(1) tspan(end)];
      obj.robot = robot;
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
    
    function [w,dw] = wrench(obj,t,kinsol,F)
      % computes the total wrench and its gradient w.r.t q and F
      % @param t      -- A double scalar to evaluate the wrench
      % @param kinsol -- The kinematics tree, returned from doKinematics
      % @param F      -- A matrix of size obj.force_size. The force parameter
      % @retval w     -- A 6 x 1 double vector. The total wrench 
      % @retval dw     -- A 6 x (nq+prod(obj.force_size)) matrix. The gradient of w w.r.t q
      % and F
      A = obj.force(t);
      [tau,dtau] = obj.torque(t,kinsol,F);
      w = [A*F(:);tau];
      dw = [zeros(3,obj.robot.getNumDOF()) A;dtau];
    end
    
    function [c,dc] = eval(obj,t,kinsol,F)
      % computes the constraint and its gradient.
      % @param t       -- A scalar. The time to evaluate the constraint
      % @param kinsol  -- kinematics tree returned from doKinematics function.
      % @param F       -- A double matrix of obj.force_size. The contact forces parameters
      % @retval c      -- A obj.num_constraint x 1 vector, the constraint value.
      % @retval dc     -- A obj.num_constraint x prod(obj.force_size) matrix. The gradient of c w.r.t F.
      [c,dc_val] = obj.evalSparse(t,kinsol,F);
      [iCfun,jCvar,m,n,nnz] = obj.evalSparseStructure(t);
      dc = sparse(iCfun,jCvar,dc_val,m,n,nnz);
    end
  end
  
  methods(Access = protected)
    function flag = checkForceSize(obj,F)
       F_size = size(F);
       flag = isnumeric(F) && length(F_size) == 2 && F_size(1) == obj.force_size(1) && F_size(2) == obj.force_size(2);
    end
  end
  
  methods(Abstract)
    [c,dc_val] = evalSparse(obj,t,kinsol,F);
      % This function evaluates the constraint and its non-zero entries in the sparse
      % gradient matrix.
      % @param t       - A scalar, the time to evaluate constraint
      % @param kinsol  - kinematics tree returned from doKinematics function
      % @param F       - A double matrix of obj.force_size. The contact forces parameter
      % @retval c      - A double column vector, the constraint value. The size of the
      % vector is obj.getNumConstraint(t) x 1
      % @retval dc_val - A double column vector. The nonzero entries of constraint gradient w.r.t F
    [iCfun,jCvar,m,n,nnz] = evalSparseStructure(obj,t);
      % This function returns the sparsity structure of the constraint gradient.
      % sparse(iCfun,jCvar,dc,m,n,nnz) is the actual gradient matrix
      % @retval iCfun   -- A num_pts x 1 double vector. The row index of the nonzero entries
      % @retval jCvar   -- A num_pts x 1 double vector. The column index of the nonzero entries
      % @retval m       -- A scalar. The total rows of the gradient matrix
      % @retval n       -- A scalar. The total columns of the gradient matrix
      % @retval nnz     -- A scalar. The maximum non-zero entries in the sparse matrix.
    [tau,dtau] = torque(obj,t,kinsol,F);
      % Compute the total torque from contact position and contact force, together with
      % its gradient
      % @param F       -- a matrix. The contact force parameter 
      % @retval tau    -- a 3 x 1 double vector, the total torque around origin.
      % @retval dtau   -- a 3 x (nq+prod(obj.force_size)) double matrix. The gradient of tau
    A = force(obj,t);
     % Compute total force from all the contact force parameters F. The total force is a
      % linear (sparse) transformation from F. total_force = A*F(:)
      % @param t    -- A double scalar. The time to evaluate force
      % @param F    -- A double matrix of obj.force_size. F is the force paramter
      % @param A    -- A 3 x prod(obj.force_size) double matrix.
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
    [pos,J] = contactPosition(obj,t,kinsol)
      % Compute the contact position and its gradient w.r.t q
      % @param t        -- A double scalar, the time to evaluate the contact position
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A matrix with 3 rows. pos(:,i) is the i'th contact position
      % @retval J       -- A matrix of size prod(size(pos)) x nq. The gradient of pos
      % w.r.t q
  end
end