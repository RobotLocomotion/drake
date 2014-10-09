classdef PostureConstraint<RigidBodyConstraint
  % A bounding box constraint on robot posture
  properties(SetAccess = protected)
    lb % The lower bound of posture
    ub % The upper bound of posture
    joint_limit_min0;% The default lower bound of the  posture of the robot
    joint_limit_max0;% The default upper bound of the  posture of the robot
  end
  
  methods
    function obj = PostureConstraint(robot,tspan)
		% @param robot                      -- A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator object
      if(nargin < 2)
        tspan = [-inf inf];
      end
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.PostureConstraintCategory,robot,tspan);
			[obj.joint_limit_min0,obj.joint_limit_max0] = robot.getJointLimits();
      obj.lb = obj.joint_limit_min0;
      obj.ub = obj.joint_limit_max0;
      obj.type = RigidBodyConstraint.PostureConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.PostureConstraintType,robot.getMexModelPtr,tspan);
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
    
    function obj = setJointLimits(obj,joint_ind,joint_min,joint_max)
		  % @param joint_ind   A column vector. The indices of the joints whose joint limits are to be set
			% @param joint_min   A column vector of the same size as joint_ind. The lower bound of the joints are max([robot.joint_limit_min(joint_ind) joint_min]),[],2);
			% @param joint_max   A column vector of the same size as joint_ind. The upper bound of the joints are min([robot.joint_limit_max(joint_ind) joint_max]),[],2);
      if obj.mex_ptr~=0
        obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'bounds',joint_ind,joint_min,joint_max);
      end
      if(any(~isnumeric(joint_ind)))
        error('The joint index should all be numerical')
      end
      if(any(joint_ind<0)||any(joint_ind>obj.robot.getNumPositions))
        error('The joint index must be within [1 nq]')
      end
      if(any(size(joint_ind)~=size(joint_min))||any(size(joint_ind)~=size(joint_max)))
        error('The size of input arguments do not match');
      end
      if(any(joint_min>joint_max))
        error('Joint min must be no larger than joint max');
      end
      obj.lb(joint_ind) = max([obj.joint_limit_min0(joint_ind) joint_min],[],2);
      obj.ub(joint_ind) = min([obj.joint_limit_max0(joint_ind) joint_max],[],2);
      if(any(obj.lb>obj.ub))
        error('Either the joint_max is smaller than the robot default joint min, or the joint min is larger than the robot default joint max');
      end
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
    
    function cnstr = generateConstraint(obj,t)
      % generate a BoundingBoxConstraint on the robot posture if t is a valid time
      % or if no time is given
      if nargin < 2, t = obj.tspan(1); end;
      if(obj.isTimeValid(t))
        cnstr = {BoundingBoxConstraint(obj.lb,obj.ub)};
      else
        cnstr = {};
      end
    end
  end
end
