classdef KinematicConstraint<Constraint
  properties
    tspan % a 1x2 vector
    num_constraint
    robot
    mex_ptr
  end
  
  properties(Constant)
    WorldCoMConstraint = 0;
    WorldPositionConstraint = 1;
    WorldQuatConstraint = 2;
    WorldEulerConstraint = 3;
    WorldGazeOrientConstraint = 4;
    WorldGazeDirConstraint = 5;
    WorldGazeTargetConstraint = 6;
  end
  
  
  methods
    function obj = KinematicConstraint(robot,tspan)
      obj = obj@Constraint(Constraint.KinematicConstraintType);
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
    
    function n = getNumConstraint(obj,t)
      if(obj.isTimeValid(t))
        n = obj.num_constraint;
      else
        n = 0;
      end
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
    
  end
  
  methods(Abstract)
    [c,dc] = eval(obj,t,kinsol)
    [lb,ub] = bounds(obj,t)
    name_str = name(obj,t)
    ptr = constructPtr(varargin);
  end
end
