classdef EulerConstraint < SingleTimeKinematicConstraint
  % constraint the roll, pitch, yaw angles (in intrinsic z-y'-x'' order)
  % to be within the bounding box [lb ub];
  properties(SetAccess = protected)
    ub
    lb
    null_constraint_rows
    avg_rpy
  end
  
  methods(Abstract,Access = protected)
    [rpy,J] = evalrpy(obj,kinsol);
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      [rpy,J] = evalrpy(obj,kinsol);
      c = rpy(~obj.null_constraint_rows);
      c = angleDiff(obj.avg_rpy,c)+obj.avg_rpy;
      dc = J(~obj.null_constraint_rows,:);
    end
  end
  
  methods
    function obj = EulerConstraint(robot,lb,ub,tspan)
      if(nargin == 3)
        tspan = [-inf, inf];
      end
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      typecheck(lb,'double');
      typecheck(ub,'double');
      sizecheck(lb,[3,1]);
      sizecheck(ub,[3,1]);
      lb(isnan(lb)) = -inf;
      ub(isnan(ub)) = inf;
      obj.null_constraint_rows = isinf(lb)&isinf(ub);
      lb = lb(~obj.null_constraint_rows);
      ub = ub(~obj.null_constraint_rows);
      if(any(lb>ub))
        error('Drake:EulerConstraint: lb must be no larger than ub');
      end
      obj.lb = lb;
      obj.ub = ub;
      obj.avg_rpy = (obj.lb+obj.ub)/2;
      obj.num_constraint = sum(~obj.null_constraint_rows);
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = obj.lb;
        ub = obj.ub;
      else
        lb = [];
        ub = [];
      end
    end
  end
end