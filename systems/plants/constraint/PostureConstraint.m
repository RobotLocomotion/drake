classdef PostureConstraint<Constraint
  % A bounding box constraint on robot posture
  % @param robot                      -- A robot
  % @param lb                         -- The lower bound of posture
  % @param ub                         -- The upper bound of posture
  % @param joint_limit_min0           -- The default lower bound of the
  %                                      posture of the robot
  % @param joint_limit_max0           -- The default upper bound of the
  %                                      posture of the robot
  properties(SetAccess = protected)
    robot
    tspan
    lb
    ub
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
      obj.lb = obj.joint_limit_min0;
      obj.ub = obj.joint_limit_max0;
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
      obj.lb(joint_ind) = max([obj.joint_limit_min0(joint_ind) joint_min],[],2);
      obj.ub(joint_ind) = min([obj.joint_limit_max0(joint_ind) joint_max],[],2);
    end
    
    function [lb,ub] = bounds(obj,t)
      if obj.isTimeValid(t)
        lb = obj.lb;
        ub = obj.ub;
      else
        lb = obj.joint_limit_min0;
        ub = obj.joint_limit_max0;
      end
    end
  end
end