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
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      [axis_ends,daxis_ends] = forwardKin(obj.robot,kinsol,obj.body,[obj.gaze_origin obj.gaze_origin+obj.axis],0);
      world_axis = axis_ends(:,2)-axis_ends(:,1);
      dworld_axis = daxis_ends(4:6,:)-daxis_ends(1:3,:);
      dir = obj.target-axis_ends(:,1);
      ddir = -daxis_ends(1:3,:);
      dir_norm = norm(dir);
      dir_normalized = dir/dir_norm;
      ddir_normalized = (eye(3)*dir_norm^2-dir*dir')/(dir_norm^3)*ddir;
      c = world_axis'*dir_normalized-1;
      dc = dir_normalized'*dworld_axis+world_axis'*ddir_normalized;
    end
  end
  
  methods
    function obj = WorldGazeTargetConstraint(robot,body,axis,target,gaze_origin,conethreshold,tspan)
      if(nargin == 6)
        tspan = [-inf inf];
      end
      obj = obj@GazeTargetConstraint(robot,axis,target,gaze_origin,conethreshold,tspan);
      obj.body = obj.robot.parseBodyOrFrameID(body);
      obj.body_name = obj.robot.getBodyOrFrameName(obj.body);
      obj.type = RigidBodyConstraint.WorldGazeTargetConstraintType;
      if(robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file'))
        ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldGazeTargetConstraintType,robot.getMexModelPtr,body,axis,target,gaze_origin,conethreshold,tspan);
        obj.mex_ptr = ptr;
      end
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('%s conic gaze target constraint at time %10.4f',obj.body_name,t)};
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
