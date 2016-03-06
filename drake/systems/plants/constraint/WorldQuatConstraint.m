classdef WorldQuatConstraint < QuatConstraint
% Constrain the body satisfies 2*(quat'*quat_des)^2-1 in [cos(tol), 1];
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             A scalar. The index of the body
% @param quat_des         A quaternion. The desired body orientation 
% @param tol              A nonnegative scalar between [0 pi]. tol is the maximum angle of
%                         the allowable rotation that would rotate actual quaternion to quat_des
  properties(SetAccess = protected)
    body
    quat_des
    body_name
  end
  
  methods(Access = protected)
    function [orient_prod, dorient_prod] = evalOrientationProduct(obj,kinsol)
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,[0;0;0],2);
      quat = pos(4:7,1);
      dquat = J(4:7,:);
      orient_prod = quat'*obj.quat_des;
      dorient_prod = obj.quat_des'*dquat;
    end
  end
  
  methods
    function obj = WorldQuatConstraint(robot,body,quat_des,tol,tspan)
      if(nargin == 4)
        tspan = [-inf,inf];
      end
      obj = obj@QuatConstraint(robot,tol,tspan);
      sizecheck(quat_des,[4,1]);
      if(any(isinf(quat_des)|isnan(quat_des)))
        error('Drake:orientConstraint:quat_des cannot have nan or inf entries');
      end
      obj.quat_des = quat_des./norm(quat_des);
      obj.body = obj.robot.parseBodyOrFrameID(body);
      obj.body_name = obj.robot.getBodyOrFrameName(obj.body);
      obj.type = RigidBodyConstraint.WorldQuatConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldQuatConstraintType,robot.getMexModelPtr,body,quat_des,tol,tspan);
      end
    end
    
    
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('%s quaternion constraint at time %10.4f',obj.body_name,t)};
      else
        name_str = [];
      end
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(1,obj.body);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
