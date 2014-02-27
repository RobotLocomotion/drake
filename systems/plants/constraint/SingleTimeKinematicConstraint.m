classdef SingleTimeKinematicConstraint < RigidBodyConstraint
  % An abstract class. Its eval function take a single time as input, the
  % constraint is enforced at that time only
  % @param num_constraint    -- An int scalar. The number of nonlinear constraints
  properties(SetAccess = protected,GetAccess = protected)
    num_constraint
  end
  
  methods
    function obj = SingleTimeKinematicConstraint(robot,tspan)
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.SingleTimeKinematicConstraintCategory,robot,tspan);
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
    
    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end
    
  end
  methods(Abstract)
    [c,dc] = eval(obj,t,kinsol);
    [lb,ub] = bounds(obj,t)
    name_str = name(obj,t)
  end
end
