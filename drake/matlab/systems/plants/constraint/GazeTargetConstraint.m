classdef GazeTargetConstraint < GazeConstraint
  properties(SetAccess = protected)
    target % a 3x1 vector, the target position in the world frame
    gaze_origin % a 3x1 vector, the origin of the gaze in the body frame
  end
  
  methods
    function obj = GazeTargetConstraint(robot,axis,target,gaze_origin,conethreshold,tspan)
      if(nargin == 5)
        tspan = [-inf inf];
      end
      % if conethreshold = [], then it means there is no
      % constraint on the conethreshold
      obj = obj@GazeConstraint(robot,axis,conethreshold,tspan);
      typecheck(target,'double');
      sizecheck(target,[3,1]);
      if(any(isinf(target))||any(isnan(target)))
        error('Drake:GazeTargetConstraint: target must be a 3x1 vector, inf or nan are note acceptted');
      end
      obj.target = target;
      typecheck(gaze_origin,'double');
      sizecheck(gaze_origin,[3,1]);
      if(any(isinf(gaze_origin))||any(isnan(gaze_origin)))
        error('Drake:GazeTargetConstraint: gaze origin must be a 3x1 vector, inf or nan are note acceptted');
      end
      obj.gaze_origin = gaze_origin;
      obj.num_constraint = 1;
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = cos(obj.conethreshold)-1;
        ub = 0;
      else
        lb = [];
        ub = [];
      end
    end
  end
end