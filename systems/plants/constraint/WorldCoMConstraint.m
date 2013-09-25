classdef WorldCoMConstraint < PositionConstraint
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param lb, ub           Both are 3x1 vectors. [lb, ub]
%                         represents the bounding box for the 
%                         position of Center of Mass in the world frame
% @param tspan            OPTIONAL argument. A 1x2 vector
  properties
    body
    body_name
  end
  
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      [pos,J] = getCOM(obj.robot,kinsol);
    end
  end
  
  methods
    function obj = WorldCoMConstraint(robot,lb,ub,tspan)
      if(nargin == 3)
        tspan = [-inf inf];
      end
      ptr = constructPtrWorldCoMConstraintmex(robot.getMexModelPtr,lb,ub,tspan);
      obj = obj@PositionConstraint(robot,[0;0;0],lb,ub,tspan);
      obj.body_name = 'CoM';
      obj.body = 0;
      obj.mex_ptr = ptr;
    end
    
    
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = cell(obj.num_constraint,1);
        constraint_idx = 1;
        if(~obj.null_constraint_rows(1))
          name_str{constraint_idx} = sprintf('CoM x');
          if(~isempty(t))
            name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
          end
          constraint_idx = constraint_idx+1;
        end
        if(~obj.null_constraint_rows(2))
          name_str{constraint_idx} = sprintf('CoM y');
          if(~isempty(t))
            name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
          end
          constraint_idx = constraint_idx+1;
        end
        if(~obj.null_constraint_rows(3))
          name_str{constraint_idx} = sprintf('CoM z');
          if(~isempty(t))
            name_str{constraint_idx} = sprintf('%s at time %10.4f',name_str{constraint_idx},t);
          end
        end
      else
        name_str = [];
      end
    end
    
    function ptr = constructPtr(varargin)
      ptr = constructPtrWorldCoMConstraintmex(varargin{:});
    end
  end
end
