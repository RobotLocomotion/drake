classdef WorldEulerConstraint <EulerConstraint
% constraint the roll, pitch, yaw angles (in intrinsic z-y'-x'' order)
% to be within the bounding box [lb ub];
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             A scalar, the index of the body
% @param lb ub            Both are 3x1 vectors.
  properties(SetAccess = protected)
    body
    body_name
  end

  methods(Access = protected)
    function [rpy,J] = evalrpy(obj,kinsol)
      [pos,dpos] = forwardKin(obj.robot,kinsol,obj.body,[0;0;0],1);
      rpy = pos(4:6);
      J = dpos(4:6,:);
    end
  end

  methods
    function obj = WorldEulerConstraint(robot,body,lb,ub)
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldEulerConstraintType,robot.getMexModelPtr,body,lb,ub);
      obj = obj@EulerConstraint(robot,lb,ub);
      obj.body = obj.robot.parseBodyOrFrameID(body);
      obj.body_name = obj.robot.getBodyOrFrameName(obj.body);
      obj.type = RigidBodyConstraint.WorldEulerConstraintType;
      obj.mex_ptr = ptr;
    end



    function name_str = name(obj,t)
      name_str = cell(obj.num_constraint,1);
      time_str = '';
      if(~isempty(t))
        time_str = sprintf('at time %5.2f',t);
      end
      constraint_idx = 1;
      if(~obj.null_constraint_rows(1))
        name_str{constraint_idx} = sprintf('%s roll %s',obj.body_name,time_str);
        constraint_idx = constraint_idx+1;
      end
      if(~obj.null_constraint_rows(2))
        name_str{constraint_idx} = sprintf('%s pitch %s',obj.body_name,t);
        constraint_idx = constraint_idx+1;
      end
      if(~obj.null_constraint_rows(3))
        name_str{constraint_idx} = sprintf('%s yaw %s',obj.body_name,t);
      end
    end

    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(1,obj.body);
      if isa(obj.robot,'TimeSteppingRigidBodyManipulator')
        joint_idx = vertcat(obj.robot.getManipulator().body(joint_path).position_num)';
      else
        joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
      end
    end
  end
end
