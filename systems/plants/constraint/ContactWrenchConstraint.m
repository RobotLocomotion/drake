classdef ContactWrenchConstraint < RigidBodyConstraint
  % constrain the contact forces
  % @param tspan   - A 1 x 2 vector. The time span of the constraint being active
  % @param robot   - A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
  % @param num_constraint   - A scalar. The number of constraints. 
  % @param mex_ptr          - A DrakeConstraintMexPointer
  % @param force_size       - A 1 x 2 matrix. The size of the force matrix.
  properties(SetAccess = protected)
    tspan % a 1x2 vector
    num_constraint;
    robot
    mex_ptr
    force_size;
  end
  
  methods
    function obj = ContactWrenchConstraint(robot,tspan)
      obj = RigidBodyConstraint(RigidBodyConstraint.ContactWrenchConstraintCategory);
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
    
  end
  
  methods(Access = protected)
    function flag = checkForceSize(obj,F)
       F_size = size(F);
       flag = isnumeric(F) && length(F_size) == 2 && F_size(1) == obj.force_size(1) && F_size(2) == obj.force_size(2);
    end
  end
  
  methods(Abstract)
    [c,dc] = eval(obj,t,F);
    [tau,dtau] = torque(obj,t,kinsol,F); % This function computes the torque of the contact force
    [iFfun,jFvar,m,n] = force(obj,F);
    [lb,ub] = bounds(obj,t);
    name_str = name(obj,t);
  end
end