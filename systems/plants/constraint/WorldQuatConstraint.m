classdef WorldQuatConstraint < QuatConstraint
% Constrain the body satisfies (quat'*quat_des)^2 in [1-tol, tol];
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             A scalar. The index of the body
% @param quat_des         A quaternion. The desired body orientation 
% @param tol              A nonnegative scalar. tol = sin(angle)^2 where
%                         'angle' is the angle in the angle-axis
%                         representation of the rotation from quat_des to
%                         the actual orientation
  properties
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
      ptr = constructPtrWorldQuatConstraintmex(robot.getMexModelPtr,body,quat_des,tol,tspan);
      obj = obj@QuatConstraint(robot,tol,tspan);
      sizecheck(quat_des,[4,1]);
      if(any(isinf(quat_des)|isnan(quat_des)))
        error('Drake:orientConstraint:quat_des cannot have nan or inf entries');
      end
      obj.quat_des = quat_des./norm(quat_des);
      if(isnumeric(body))
        sizecheck(body,[1,1]);
        obj.body = body;
      elseif(typecheck(body,'char'))
        obj.body = robot.findLinkInd(body);
      elseif(typecheck(body,'RigidBody'))
        obj.body = robot.findLinkInd(body.linkname);
      else
        error('Drake:WorldQuatConstraint:Body must be either the link name or the link index');
      end
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      obj.mex_ptr = ptr;
    end
    
    
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('%s quaternion constraint at time %10.4f',obj.body_name,t)};
      else
        name_str = [];
      end
    end
    
    function ptr = constructPtr(varargin)
      ptr = constructPtrWorldQuatConstraintmex(varargin{:});
    end
  end
end
