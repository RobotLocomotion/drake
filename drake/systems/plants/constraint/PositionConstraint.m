classdef PositionConstraint < SingleTimeKinematicConstraint
  properties(SetAccess = protected)
    ub
    lb
    null_constraint_rows
    pts
    n_pts
  end

  methods(Abstract,Access = protected)
    [pos,J] = evalPositions(obj,kinsol)
    cnst_names = evalNames(obj,t)
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      [pos,J] = evalPositions(obj,kinsol); 
      c = pos(~obj.null_constraint_rows);
      dc = J(~obj.null_constraint_rows,:);
    end
  end
  
  methods(Access = protected)
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
  
  methods
    function obj = PositionConstraint(robot,pts,lb,ub,tspan)
      if(nargin == 4)
        tspan = [-inf,inf];
      end
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      if(size(pts,1)~=3)
        error('Incorrect size');
      end
      obj.pts = pts;
      obj.n_pts = size(obj.pts,2);
      obj = setConstraintBounds(obj,lb,ub);
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

    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        cnst_names = evalNames(obj,t);
        name_str = cnst_names(~obj.null_constraint_rows);
      else
        name_str = [];
      end
    end
  end
end

