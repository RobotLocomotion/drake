classdef MultipleTimeKinematicConstraint < Constraint
  % A abstract class, that its eval function takes multiple time points as
  % the input, instead of being evluated at a single time.
  properties
    tspan % a 1x2 vector
    robot
    mex_ptr
  end
  
  properties(Constant)
    WorldFixedPositionConstraint = 1;
    WorldFixedOrientConstraint = 2;
    WorldFixedBodyPostureConstraint = 3;
  end
    
  methods
    function obj = MultipleTimeKinematicConstraint(robot,tspan)
      obj = obj@Constraint(Constraint.MultipleTimeKinematicConstraintType);
      if(nargin<2)
        tspan = [-inf,inf];
      end
      if(isempty(tspan))
        tspan = [-inf,inf];
      end
      if(tspan(1)>tspan(end))
        error('tspan(1) should be no larger than tspan(end)')
      end
      obj.tspan = [tspan(1) tspan(end)];
      obj.robot = robot;
    end
    
    function flag = isTimeValid(obj,t)
      n_breaks = size(t,2);
      if(n_breaks <=1)
        error('Drake:WorldFixedPositionConstraint: t must have more than 1 entry');
      end
      flag = all(t>=obj.tspan(1)&t<=obj.tspan(end));
    end
    
    function tspan = getTspan(obj)
      tspan = obj.tspan;
    end
    
  end
  
  methods(Abstract)
    % t is an array instead of a scalar
    num = getNumConstraint(obj,t);
    [c,dc] = eval(obj,t,q);
    [lb,ub] = bounds(obj,t)
    name_str = name(obj,t)
    ptr = constructPtr(varargin);
  end
end