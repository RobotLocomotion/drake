classdef WorldQuatConstraint < QuatConstraint
  % Constrain the body satisfies (quat'*quat_des)^2 in [1-tol, tol];
  % Documentation here
  properties
    body
    quat_des
    body_name
  end
  
  methods
    function obj = WorldQuatConstraint(robot,body,quat_des,tol,tspan)
      if(nargin == 4)
        tspan = [-inf,inf];
      end
      ptr = constructPtrWorldQuatConstraintmex(robot.getMexModelPtr,tspan,body,quat_des,tol);
      obj = obj@QuatConstraint(robot,tspan,tol);
      sizecheck(quat_des,[4,1]);
      if(any(isinf(quat_des)|isnan(quat_des)))
        error('Drake:orientConstraint:quat_des cannot have nan or inf entries');
      end
      obj.quat_des = quat_des./norm(quat_des);
      if(typecheck(body,'char'))
        obj.body = robot.findLinkInd(body);
      elseif(typecheck(body,'double'))
        sizecheck(body,[1,1]);
        obj.body = body;
      elseif(typecheck(body,'RigidBody'))
        obj.body = robot.findLinkInd(body.linkname);
      else
        error('Drake:WorldOrientConstraint:Body must be either the link name or the link index');
      end
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      obj.mex_ptr = ptr;
    end
    
    function [orient_prod, dorient_prod] = evalOrientationProduct(obj,kinsol)
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,[0;0;0],2);
      quat = pos(4:7,1);
      dquat = J(4:7,:);
      orient_prod = quat'*obj.quat_des;
      dorient_prod = obj.quat_des'*dquat;
    end
    
    function name = evalName(obj,t)
      if(obj.isTimeValid(t))
        name = {sprintf('%s quaternion constraint at time %10.4f',obj.body_name,t)};
      else
        name = [];
      end
    end
    
  end
end
