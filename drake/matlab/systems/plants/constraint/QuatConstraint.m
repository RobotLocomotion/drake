classdef QuatConstraint <SingleTimeKinematicConstraint
  % Constrain the quaternion to satisfy the following conditions: 
  % 2*(quat'*quat_des)^2-1 in [cos(tol) 1]
  properties(SetAccess = protected)
    tol
  end
  
  methods(Abstract,Access = protected)
    [orient_prod,dorient_prod] = evalOrientationProduct(obj,kinsol)
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      [orient_prod, dorient_prod] = evalOrientationProduct(obj,kinsol);
      c = 2*orient_prod^2-1;
      dc = 4*(orient_prod)*dorient_prod;
    end
  end
  
  methods
    function obj = QuatConstraint(robot,tol,tspan)
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      sizecheck(tol,[1,1]);
      if(tol<0||tol>pi)
        error('Drake:QuatConstraint:Tol must be within the range [0 pi]');
      end
      obj.tol = tol;
      obj.num_constraint = 1;
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = cos(obj.tol);
        ub = 1;
      else
        lb = [];
        ub = [];
      end
    end
    
  end
end
