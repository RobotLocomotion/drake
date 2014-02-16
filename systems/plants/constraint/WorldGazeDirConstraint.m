classdef WorldGazeDirConstraint < GazeDirConstraint
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             The index of the body
% @param axis             A 3x1 unit vector, in the body frame
% @param dir              A 3x1 unit vector, in the world frame
% @param conethreshold    A scalar in [0 pi], default is 0
% @param tspan            OPTIONAL argument. A 1x2 vector
% The body axis lies within a cone, with the cone
% aperture being 2*conethreshold, the cone axis being dir
  properties(SetAccess = protected)
    body;
    body_name;
  end
  
  
  methods  
    function obj = WorldGazeDirConstraint(robot,body,axis,dir,conethreshold,tspan)
      if(nargin == 5)
        tspan = [-inf inf];
      end
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldGazeDirConstraintType,robot.getMexModelPtr,body,axis,dir,conethreshold,tspan);
      obj = obj@GazeDirConstraint(robot,axis,dir,conethreshold,tspan);
      if(isnumeric(body))
        sizecheck(body,[1,1]);
        obj.body = body;
      elseif(typecheck(body,'char'))
        obj.body = robot.findLinkInd(body);
      elseif(typecheck(body,'RigidBody'))
        obj.body = robot.findLinkInd(body.linkname);
      else
        error('Drake:WorldPositionConstraint:Body must be either the link name or the link index');
      end
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      obj.type = RigidBodyConstraint.WorldGazeDirConstraintType;
      obj.mex_ptr = ptr;
    end
    
    

    function [c,dc] = eval(obj,t,kinsol)
      if(obj.isTimeValid(t))
        [axis_pos,daxis_pos] = forwardKin(obj.robot,kinsol,obj.body,[zeros(3,1) obj.axis],0);
        axis_world = axis_pos(:,2)-axis_pos(:,1);
        daxis_world = daxis_pos(4:6,:)-daxis_pos(1:3,:);
        c = axis_world'*obj.dir-1;
        dc = obj.dir'*daxis_world;
      else
        c = [];
        dc = [];
      end
    end
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('%s conic gaze direction constraint at time %10.4f',obj.body_name,t)};
      else
        name_str = [];
      end
    end
    
  end
end