classdef PostureConstraint<Constraint
  properties
    robot
    tspan
    joint_limit_min
    joint_limit_max
    joint_limit_min0;
    joint_limit_max0;
    mex_ptr;
  end
  
  methods
    function obj = PostureConstraint(robot,tspan)
      if(nargin == 1)
        tspan = [-inf inf];
      end
      ptr = constructPtrPostureConstraintmex(robot.getMexModelPtr,tspan);
      obj = obj@Constraint(Constraint.PostureConstraintType);
      if(isempty(tspan))
        tspan = [-inf,inf];
      end
      if(tspan(end)<tspan(1))
        error('tspan(end) should be no smaller than tspan(1)')
      end
      obj.tspan = [tspan(1) tspan(end)];
      obj.robot = robot;
      [obj.joint_limit_min0,obj.joint_limit_max0] = robot.getJointLimits();
      obj.joint_limit_min = obj.joint_limit_min0;
      obj.joint_limit_max = obj.joint_limit_max0;
      obj.mex_ptr = ptr;
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
    
    function obj = setJointLimits(obj,joint_ind,joint_min,joint_max)
      constructPtrPostureConstraintmex(obj.mex_ptr,joint_ind,joint_min,joint_max);
      if(any(~isnumeric(joint_ind)))
        error('The joint index should all be numerical')
      end
      if(any(joint_ind<0)||any(joint_ind>obj.robot.getNumDOF))
        error('The joint index must be within [1 nq]')
      end
      if(any(size(joint_ind)~=size(joint_min))||any(size(joint_ind)~=size(joint_max)))
        error('The size of input arguments do not match');
      end
      if(any(joint_min>joint_max+1e-5))
        error('Joint min must be no larger than joint max');
      end
      obj.joint_limit_min(joint_ind) = max([obj.joint_limit_min0(joint_ind) joint_min],[],2);
      obj.joint_limit_max(joint_ind) = min([obj.joint_limit_max0(joint_ind) joint_max],[],2);
    end
    
    function [joint_min,joint_max] = bounds(obj,t)
      if obj.isTimeValid(t)
        joint_min = obj.joint_limit_min;
        joint_max = obj.joint_limit_max;
      else
        joint_min = obj.joint_limit_min0;
        joint_max = obj.joint_limit_max0;
      end
    end
  end
end