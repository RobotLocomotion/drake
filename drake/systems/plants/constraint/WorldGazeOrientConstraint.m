classdef WorldGazeOrientConstraint < GazeOrientConstraint
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             The index of the body
% @param axis                a 3x1 unit vector, in the body frame
% @param quat_des            a 4x1 unit vector
% @param conethreshold       a scalar in [0,pi], default is 0
% @param threshold           a scalar in [0,pi], default is pi
% @param tspan            OPTIONAL argument. A 1x2 vector
% quat_des is the nominal orientation of the body. The actual orientation
% of the body satisfies that the axis lies within a cone, with the cone
% aperture being 2*conethreshold, the cone axis being the direction that
% the body axis would be if body orientation exactly matches quat_des;
% The body can also rotate around that cone axis within range [-threshold
% threshold]
  properties(SetAccess = protected)
    body
    body_name
  end
  
  methods(Access = protected)
    function [quat, dquat_dq] = evalOrientation(obj,kinsol)
      [x,J] = forwardKin(obj.robot,kinsol,obj.body,[0;0;0],2);
      quat = x(4:7);
      dquat_dq = J(4:7,:);
    end
  end
  
  methods
    function obj = WorldGazeOrientConstraint(robot,body,axis,quat_des,conethreshold,threshold,tspan)
      if(nargin == 6)
        tspan = [-inf inf];
      end
      obj = obj@GazeOrientConstraint(robot,axis,quat_des,conethreshold,threshold,tspan);
      obj.body = obj.robot.parseBodyOrFrameID(body);
      obj.body_name = obj.robot.getBodyOrFrameName(obj.body);
      obj.type = RigidBodyConstraint.WorldGazeOrientConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldGazeOrientConstraintType,robot.getMexModelPtr,body,axis,quat_des,conethreshold,threshold,tspan);
      end
    end

    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('%s conic gaze orientation constraint at time %10.4f',obj.body_name,t);...
          sprintf('%s revolute gaze orientation constraint at time %10.4f',obj.body_name,t)};
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
