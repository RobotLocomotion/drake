classdef IKconstraint
  properties
    tspan % a 1x2 vector
    num_constraint
    mex_ptr
  end
  methods
    function obj = IKconstraint(tspan)
      if(tspan(1)>tspan(end)+1e-5)
        error('tspan(1) should be no larger than tspan(end)')
      end
      obj.tspan = [tspan(1) tspan(end)];
      obj.mex_ptr = constructPtr(obj);
    end
    
    function n = getNumConstraint(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        n = obj.num_constraint;
      else
        n = 0;
      end
    end
  end
  
  methods(Abstract)
    [c,dc] = getConstraintVal(obj,t,kinsol)
    [lb,ub] = getConstraintBnds(obj,t)
    name = getConstraintName(obj,t)
    ptr = constructPtr(obj);
  end
end
