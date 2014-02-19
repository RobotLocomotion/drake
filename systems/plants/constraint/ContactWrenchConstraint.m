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
    % returns the constraint and the non-zero entries in its sparse gradient matrix
    [iCfun,jCvar,m,n,nnz] = evalSparseStructure(obj,t);
    % returns the sparsity pattern of the constraint gradient
    [tau,dtau] = torque(obj,t,kinsol,F);
    % This function computes the total torque of the contact force
    A = force(obj,t);
    % We suppose that the total force is a linear combination of F. This function returns such a (sparse) linear transformation A
    [lb,ub] = bounds(obj,t);
    % Returns the lower and upper bound of the constraint returned by eval function
    name_str = name(obj,t);
    % returns the name of each constraint returned by eval function
  end
end