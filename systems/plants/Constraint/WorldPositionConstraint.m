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
  properties
    body
    body_name
  end
  
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
    end
  end
  
  methods
    function obj = WorldPositionConstraint(robot,body,pts,lb,ub,tspan)
      if(nargin == 5)
        tspan = [-inf,inf];
      end
      ptr = constructPtrWorldPositionConstraintmex(robot.getMexModelPtr,body,pts,lb,ub,tspan);
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
      obj.mex_ptr = ptr;
    end
    
    
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = cell(obj.num_constraint,1);
        constraint_idx = 1;
        for i = 1:obj.n_pts
          if(~obj.null_constraint_rows(3*(i-1)+1))
            name_str{constraint_idx} = sprintf('%s pts(:,%d) x',obj.body_name,i);
            if(~isempty(t))
              name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
            end
            constraint_idx = constraint_idx+1;
          end
          if(~obj.null_constraint_rows(3*(i-1)+2))
            name_str{constraint_idx} = sprintf('%s pts(:,%d) y',obj.body_name,i);
            if(~isempty(t))
              name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
            end
            constraint_idx = constraint_idx+1;
          end
          if(~obj.null_constraint_rows(3*(i-1)+3))
            name_str{constraint_idx} = sprintf('%s pts(:,%d) z',obj.body_name,i);
            if(~isempty(t))
              name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
            end
            constraint_idx = constraint_idx+1;
          end
        end
      else
        name_str = [];
      end
    end
    
    
    function ptr = constructPtr(varargin)
      ptr = constructPtrWorldPositionConstraintmex(varargin{:});
    end
  end
end