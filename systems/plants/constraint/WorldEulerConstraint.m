classdef WorldEulerConstraint <EulerConstraint
% constraint the roll, pitch, yaw angles (in intrinsic z-y'-x'' order)
% to be within the bounding box [lb ub];
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator  
% @param body             A scalar, the index of the body
% @param lb ub            Both are 3x1 vectors.
% @param tspan            OPTIONAL argument. A 1x2 vector
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
    function obj = WorldEulerConstraint(robot,body,lb,ub,tspan)
      if(nargin ==4)
        tspan = [-inf inf];
      end
      ptr = constructPtrWorldEulerConstraintmex(robot.getMexModelPtr,body,lb,ub,tspan);
      obj = obj@EulerConstraint(robot,lb,ub,tspan);
      if(isnumeric(body))
        sizecheck(body,[1,1]);
        obj.body = body;
      elseif(typecheck(body,'char'))
        obj.body = robot.findLinkInd(body);
      elseif(typecheck(body,'RigidBody'))
        obj.body = robot.findLinkInd(body.linkname);
      else
        error('Drake:WorldEulerConstraint:Body must be either the link name or the link index');
      end
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      obj.mex_ptr = ptr;
    end
    
    
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = cell(obj.num_constraint,1);
        constraint_idx = 1;
        if(~obj.null_constraint_rows(1))
          name_str{constraint_idx} = sprintf('%s roll',obj.body_name);
          if(~isempty(t))
            name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
          end
          constraint_idx = constraint_idx+1;
        end
        if(~obj.null_constraint_rows(2))
          name_str{constraint_idx} = sprintf('%s pitch',obj.body_name);
          if(~isempty(t))
            name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
          end
          constraint_idx = constraint_idx+1;
        end
        if(~obj.null_constraint_rows(3))
          name_str{constraint_idx} = sprintf('%s yaw',obj.body_name);
          if(~isempty(t))
            name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
          end
        end
      else
        name_str = [];
      end
    end
    
    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      updatePtrWorldEulerConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end
    
    function ptr = constructPtr(varargin)
      ptr = constructPtrWorldEulerConstraintmex(varargin{:});
    end
  end
end