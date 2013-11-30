classdef SingleTimeKinematicConstraint < RigidBodyConstraint
  % An abstract class. Its eval function take a single time as input, the
  % constraint is enforced at that time only
  properties(SetAccess = protected)
    tspan % a 1x2 vector
    num_constraint
    robot
    mex_ptr
  end
  
  methods
    function obj = SingleTimeKinematicConstraint(robot,tspan)
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.SingleTimeKinematicConstraintCategory);
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
    
  end
  methods(Abstract)
    [c,dc] = eval(obj,t,kinsol);
    [lb,ub] = bounds(obj,t)
    name_str = name(obj,t)
    obj = updateRobot(obj,r);
  end
end
