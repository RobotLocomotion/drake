classdef GazeDirConstraint < GazeConstraint
  properties
    dir % a 3x1 vector in the world frame, the deried gaze direction
  end
  
  methods(Access = protected)
    [quat,dquat_dq] = evalOrientation(obj,kinsol);
  end
  
  methods
    function obj = GazeDirConstraint(robot,axis,dir,conethreshold,tspan)
      if(nargin == 4)
        tspan = [-inf inf];
      end
      obj = obj@GazeConstraint(robot,axis,conethreshold,tspan);
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
    
    function [c,dc] = eval(obj,t,kinsol)
      if(obj.isTimeValid(t))
        [quat,dquat] = evalOrientation(obj,kinsol);
        quat_des = quatTransform(obj.axis,obj.dir);
        [axis_err,daxis_err] = quatDiffAxisInvar(quat,quat_des,obj.axis);
        daxis_err_dq = daxis_err(1:4)*dquat;
        c = axis_err;
        dc = daxis_err_dq;
      else
        c = [];
        dc = [];
      end
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