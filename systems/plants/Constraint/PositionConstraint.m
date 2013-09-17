classdef PositionConstraint < KinematicConstraint
  properties
    ub
    lb
    null_constraint_rows
    pts
    n_pts
  end

  methods(Abstract,Access = protected)
    [pos,J] = evalPositions(obj,kinsol)
  end

  methods
    function obj = PositionConstraint(robot,pts,lb,ub,tspan)
      if(nargin == 4)
        tspan = [-inf,inf];
      end
      obj = obj@KinematicConstraint(robot,tspan);
      if(size(pts,1)~=3)
        error('Incorrect size');
      end
      obj.pts = pts;
      obj.n_pts = size(obj.pts,2);
      obj = setConstraintBounds(obj,lb,ub);
    end

    function [c,dc] = eval(obj,t,kinsol)
      if(obj.isTimeValid(t))
        [pos,J] = evalPositions(obj,kinsol); 
        c = pos(~obj.null_constraint_rows);
        dc = J(~obj.null_constraint_rows,:);
      else
        c = [];
        dc = [];
      end
    end

    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = obj.lb;
        ub = obj.ub;
      else
        lb = [];
        ub = [];
      end
    end

    function obj = setConstraintBounds(obj, lb, ub)
      rows = size(lb,1);
      if (size(lb,2)==1&&obj.n_pts~=1); lb = bsxfun(@times,lb,ones(1,obj.n_pts)); end
      if (size(ub,2)==1&&obj.n_pts~=1); ub = bsxfun(@times,ub,ones(1,obj.n_pts)); end
      if(any(size(lb) ~=[rows,obj.n_pts])||any(size(ub)~=[rows,obj.n_pts]));
        error('Size incorrect');
      end
      if(rows ~=3)
        error('Drake:PositionConstraint:The rows of ub and lb must be 3');
      end
      if(any(lb>ub))
        error('Drake:PositionConstraint: lb must be no larger than ub');
      end
      lb(isnan(lb)) = -inf;
      ub(isnan(ub)) = inf;
      obj.null_constraint_rows = isinf(lb(:))&isinf(ub(:));
      obj.lb = lb(~obj.null_constraint_rows);
      obj.ub = ub(~obj.null_constraint_rows);
      obj.num_constraint = sum(~obj.null_constraint_rows);
    end
  end
end

