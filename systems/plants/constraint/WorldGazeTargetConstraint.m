classdef WorldGazeTargetConstraint < GazeTargetConstraint
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             The index of the body
% @param axis             A 3x1 unit vector in the body frame
% @param target           A 3x1 point in the world frame
% @param gaze_origin      A 3x1 point in the body frame
% @param conethreshold    A scalar in [0,pi], default is 0
% @param tspan            OPTIONAL argument. A 1x2 vector
% The body has the orientation such that its axis is within a cone, the
% aperture of the cone being 2xconethreshold, the axis of the cone being
% the line connecting target and gaze_origin
  properties(SetAccess = protected)
    body
    body_name
  end
  
  methods
    function obj = WorldGazeTargetConstraint(robot,body,axis,target,gaze_origin,conethreshold,tspan)
      if(nargin == 6)
        tspan = [-inf inf];
      end
      ptr = constructPtrWorldGazeTargetConstraintmex(robot.getMexModelPtr,body,axis,target,gaze_origin,conethreshold,tspan);
      obj = obj@GazeTargetConstraint(robot,axis,target,gaze_origin,conethreshold,tspan);
      if(isnumeric(body))
        sizecheck(body,[1,1]);
        obj.body = body;
      elseif(typecheck(body,'char'))
        obj.body = robot.findLinkInd(body);
      elseif(typecheck(body,'RigidBody'))
        obj.body = robot.findLinkInd(body.linkname);
      else
        error('Drake:WorldGazeTargetConstraint:Body must be either the link name or the link index');
      end
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      obj.mex_ptr = ptr;
    end
    
    function [c,dc] = eval(obj,t,kinsol)
      if(obj.isTimeValid(t))
        [x,J] = forwardKin(obj.robot,kinsol,obj.body,obj.gaze_origin,2);
        gaze_vec = obj.target-x(1:3);
        len_gaze_vec = norm(gaze_vec);
        dlen_gaze_vec = -gaze_vec'/len_gaze_vec*J(1:3,:);
        dgaze_vec = (-J(1:3,:)*len_gaze_vec-gaze_vec*dlen_gaze_vec)/len_gaze_vec^2;
        gaze_vec = gaze_vec/len_gaze_vec;
        [quat_des,dquat_des] = quatTransform(gaze_vec,obj.axis);
        dquat_des_dq = dquat_des(:,1:3)*dgaze_vec;
        [axis_err,daxis_err] = quatDiffAxisInvar(x(4:7),quat_des,obj.axis);
        daxis_err_dq = daxis_err(1:4)*J(4:7,:)+daxis_err(5:8)*dquat_des_dq;
        c = axis_err;
        dc = daxis_err_dq;
      else
        c = [];
        dc = [];
      end
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('%s conic gaze target constraint at time %10.4f',obj.body_name,t)};
      else
        name_str = [];
      end
    end
    
    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj.mex_ptr = updatePtrWorldGazeTargetConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end
    
    function ptr = constructPtr(varargin)
      ptr = constructPtrWorldGazeTargetConstraintmex(varargin{:});
    end
  end
end