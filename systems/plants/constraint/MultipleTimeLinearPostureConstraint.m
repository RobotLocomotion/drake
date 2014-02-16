classdef MultipleTimeLinearPostureConstraint < RigidBodyConstraint
  % A linear constraint on the robot posture for multiple times
  properties(SetAccess = protected)
    robot
    tspan
    mex_ptr
  end
  
  properties(Constant)
    PostureChangeConstraint = 1;
  end
  
  methods
    function obj = MultipleTimeLinearPostureConstraint(robot,tspan)
      if(nargin<2)
        tspan = [-inf inf];
      end
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.MultipleTimeLinearPostureConstraintCategory);
      obj.robot = robot;
      if(~isnumeric(tspan))
        error('Drake:MultipeTimeLinearPostureConstraint: tspan should be numeric');
      end
      sizecheck(tspan,[1,2]);
      if(tspan(1)>tspan(2))
        error('Drake:MultipleTimeLinearPostureConstraint: tspan(1) should be no larger than tspan(2)');
      end
      obj.tspan = tspan;
    end
    
    function flag = isTimeValid(obj,t)
      diff_t = diff(t);
      if(any(diff_t)<=0)
        error('Drake:MultipleTimeLinearPostureConstraint: t must be in the ascending order');
      end
      n_breaks = size(t,2);
      if(n_breaks<=1)
        error('Drake:MultipleTimeLinearPostureConstraint: t must have more than one entry');
      end
      flag = t>=obj.tspan(1) & t<=obj.tspan(end);
    end
    
    function [c,dc] = eval(obj,t,q)
      c = obj.feval(t,q);
      [iAfun,jAvar,A] = geval(obj,t);
      num_cnst = obj.getNumConstraint(t);
      dc = sparse(iAfun,jAvar,A,num_cnst,numel(q));
    end
  end
  
  methods(Abstract)
    num = getNumConstraint(obj,t);
    c = feval(obj,t,q);
    [iAfun,jAvar,A] = geval(obj,t);
    % @retval iAfun,jAvar,A  - The sparse matrix determined by
    % sparse(iAfun,jAvar,A,num_constraint,numel(q)) is the gradient of c w.r.t q
    [lb,ub] = bounds(obj,t);
    name_str = name(obj,t);
  end
end