classdef GazeOrientConstraint < GazeConstraint
  properties(SetAccess = protected)
    threshold % the angle in radians, indicating how much the body can rotate along the axis, default is pi
    quat_des % a 4x1 vector, indicating the desired transformation from the body frame to the world frame
  end
  
  methods(Abstract,Access = protected)
      [quat, dquat_dq] = evalOrientation(obj,kinsol);
  end

  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)      
      [quat, dquat] = evalOrientation(obj,kinsol);
      [axis_err,daxis_err] = quatDiffAxisInvar(quat,obj.quat_des,obj.axis);
      daxis_err_dq = daxis_err(1:4)*dquat;
      [q_diff,dq_diff] = quatDiff(quat,obj.quat_des);
      dq_diff_dq = dq_diff(:,1:4)*dquat;
      c = [axis_err;q_diff(1)];
      dc = [daxis_err_dq;dq_diff_dq(1,:)];
    end
  end
  
  methods
    function obj = GazeOrientConstraint(robot,axis,quat_des,conethreshold,threshold,tspan)
      if(nargin  == 5)
        tspan = [-inf inf];
      end
      obj = obj@GazeConstraint(robot,axis,conethreshold,tspan);
      sizecheck(quat_des,[4,1]);
      if(any(isinf(quat_des)|isnan(quat_des)))
        error('Drake:GazeOrientConstraint:quat_des cannot have nan or inf entries');
      end
      len_quat_des = norm(quat_des);
      if(len_quat_des == 0)
        error('Drake:GazeOrientConstraint: quat_des must be a nonzero vector');
      end
      obj.quat_des = quat_des./len_quat_des;
      if(isempty(threshold))
        threshold = pi;
      end
      typecheck(threshold,'double');
      sizecheck(threshold,[1,1]);
      if(threshold<0 || threshold>pi)
        error('Drake:GazeConstraint: threshold must be within [0 pi]');
      end
      obj.threshold = threshold;
      obj.num_constraint = 2;
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = [cos(obj.conethreshold)-1;cos(obj.threshold/2)];
        ub = [0;inf];
      else
        lb = [];
        ub = [];
      end
    end
  end
end
