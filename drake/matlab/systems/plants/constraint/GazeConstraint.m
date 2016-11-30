classdef GazeConstraint < SingleTimeKinematicConstraint
  properties(SetAccess = protected)
    axis % a 3x1 vector in the body frame, the gaze axis    
    conethreshold % the angle in radians, measures the angle intersected by actual gaze axis and the desired gaze axis, default is 0
  end
  
  methods
    function obj = GazeConstraint(robot,axis,conethreshold,tspan)
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      typecheck(axis,'double');
      sizecheck(axis,[3,1]);
      len_axis = norm(axis);
      if(len_axis == 0)
        error('Drake:GazeConstraint: axis must be a non zero vector');
      end
      obj.axis = axis/len_axis;
      if(nargin == 3)
        conethreshold = [];
      end
      if(isempty(conethreshold))
        conethreshold = 0;
      end
      sizecheck(conethreshold,[1,1]);
      if(conethreshold<0|| conethreshold>pi)
        error('Drake:GazeConstraint: cone threshold must be within [0 pi]');
      end
      obj.conethreshold = conethreshold;
    end
    
  end
end