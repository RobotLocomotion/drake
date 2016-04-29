classdef GazeDirConstraint < GazeConstraint
  properties(SetAccess = protected)
    dir % a 3x1 vector in the world frame, the deried gaze direction
  end


  methods
    function obj = GazeDirConstraint(robot,axis,dir,conethreshold)
      obj = obj@GazeConstraint(robot,axis,conethreshold);
      typecheck(dir,'double');
      sizecheck(dir,[3,1]);
      if(any(isinf(dir))||any(isnan(dir)))
        error('Drake:GazeDirConstraint: dir must be a 3x1 vector, inf or nan are not acceppted');
      end
      len_dir = norm(dir);
      if(len_dir == 0)
        error('Drake:GazeDirConstraint: gaze direction must be a non zero vector');
      end
      obj.dir = dir/len_dir;
      obj.num_constraint = 1;
    end


    function [lb,ub] = bounds(obj,t)
      lb = cos(obj.conethreshold)-1;
      ub = 0;
    end
  end

end
