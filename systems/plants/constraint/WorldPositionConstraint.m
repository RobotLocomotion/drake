classdef WorldPositionConstraint < PositionConstraint
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             The index of the body
% @param pts              A 3xn_pts matrix, pts(:,i) represents ith pts in
%                         the body
% @param lb, ub           Both are 3xn_pts matrices. [lb(:,i), ub(:,i)]
%                         represents the bounding box for the 
%                         position of pts(:,i) in the world frame
% @param tspan            OPTIONAL argument. A 1x2 vector
  properties(SetAccess = protected)
    body
    body_name
  end
  
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
    end
    
    function cnst_names = evalNames(obj,t)
      cnst_names = cell(3*obj.n_pts,1);
      if(isempty(t))
        time_str = '';
      else
        time_str = sprintf('at time %5.2f',t);
      end
      for i = 1:obj.n_pts
        cnst_names{3*(i-1)+1} = sprintf('%s pts(:,%d) x %s',obj.body_name,i,time_str);
        cnst_names{3*(i-1)+2} = sprintf('%s pts(:,%d) y %s',obj.body_name,i,time_str);
        cnst_names{3*(i-1)+3} = sprintf('%s pts(:,%d) z %s',obj.body_name,i,time_str);
      end
    end
  end
  
  methods
    function obj = WorldPositionConstraint(robot,body,pts,lb,ub,tspan)
      if(nargin == 5)
        tspan = [-inf,inf];
      end
      obj = obj@PositionConstraint(robot,pts,lb,ub,tspan);
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
      obj.type = RigidBodyConstraint.WorldPositionConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,robot.getMexModelPtr,body,pts,lb,ub,tspan);
      end
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(1,obj.body);
      if isa(obj.robot,'TimeSteppingRigidBodyManipulator')
        joint_idx = vertcat(obj.robot.getManipulator().body(joint_path).dofnum)';
      else
        joint_idx = vertcat(obj.robot.body(joint_path).dofnum)';
      end
    end
    
  end
end
