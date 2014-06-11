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
    
    function cnst_names = evalNames(obj,t)
      cnst_names = cell(3,1);
      if(isempty(t))
        time_str = '';
      else
        time_str = sprintf('at time %5.2f',t);
      end
        cnst_names{1} = sprintf('CoM x %s',time_str);
        cnst_names{2} = sprintf('CoM y %s',time_str);
        cnst_names{3} = sprintf('CoM z %s',time_str);
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
      obj = obj@PositionConstraint(robot,[0;0;0],lb,ub,tspan);
      if(~isempty(setdiff(robotnum,1:length(obj.robot.name))))
        error('Drake:WorldCoMConstraint: robotnum is not accepted');
      end
      obj.robotnum = robotnum;
      obj.body_name = 'CoM';
      obj.body = 0;
      obj.type = RigidBodyConstraint.WorldCoMConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldCoMConstraintType,robot.getMexModelPtr,lb,ub,tspan,robotnum);
      end
    end
    
    
    
    function obj = updateRobotnum(obj,robotnum)
      if(~isempty(setdiff(robotnum,1:length(obj.robot.name))))
        error('Drake:WorldCoMConstraint: robotnum is not accepted');
      end
      obj.robotnum = robotnum;
      if obj.mex_ptr ~=0 
        obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robotnum',robotnum);
      end
    end
    
  end
end
