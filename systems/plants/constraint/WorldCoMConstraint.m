classdef WorldCoMConstraint < PositionConstraint
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator 
% @param lb, ub           Both are 3x1 vectors. [lb, ub]
%                         represents the bounding box for the 
%                         position of Center of Mass in the world frame
% @param tspan            OPTIONAL argument. A 1x2 vector
% @param robotnum         The indices of the robot whose CoM is computed as a whole,default is 1.
  properties(SetAccess = protected)
    robotnum
    body
    body_name
  end
  
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      [pos,J] = getCOM(obj.robot,kinsol,obj.robotnum);
    end
  end
  
  methods
    function obj = WorldCoMConstraint(robot,lb,ub,tspan,robotnum)
      if(nargin<=4)
        robotnum = 1;
      end
      if(nargin <= 3)
        tspan = [-inf inf];
      end
      ptr = constructPtrWorldCoMConstraintmex(robot.getMexModelPtr,lb,ub,tspan,robotnum);
      obj = obj@PositionConstraint(robot,[0;0;0],lb,ub,tspan);
      if(~isempty(setdiff(robotnum,1:length(obj.robot.name))))
        error('Drake:WorldCoMConstraint: robotnum is not accepted');
      end
      obj.robotnum = robotnum;
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
    
    function obj = updateRobotnum(obj,robotnum)
      if(~isempty(setdiff(robotnum,1:length(obj.robot.name))))
        error('Drake:WorldCoMConstraint: robotnum is not accepted');
      end
      obj.robotnum = robotnum;
    end
    function obj = updateRobot(obj,r)
      obj.robot = r;
      updatePtrWorldCoMConstraintmex(obj.mex_ptr,'robot',obj.robot.getMexModelPtr);
    end
  end
end
