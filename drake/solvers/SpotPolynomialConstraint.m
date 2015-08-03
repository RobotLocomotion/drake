classdef SpotPolynomialConstraint < Constraint % todo: inherit from PolynomialConstraint (once it exists)
  % a polynomial constraint of the form lb <= p(x) <= ub, where p is stored as an msspoly column vector
  properties(SetAccess = protected)
    p
    dp
    p_x
  end

  methods
    function obj = SpotPolynomialConstraint(lb,ub,p_x,p)
      % @lb lower bound
      % @ub upper bound
      % @p_x simple msspoly representing the input variables
      % @p the msspoly representing the polynomial constraint
      typecheck(p,'msspoly');
      typecheck(p_x,'msspoly');
      assert(issimple(p_x)==1);
      sizecheck(lb,size(p));
      sizecheck(ub,size(p));
      nx = size(p_x,1);
      obj = obj@Constraint(lb,ub,nx,1);
      obj.p = p;
      obj.p_x = p_x;
      obj.dp = diff(p,p_x);
    end
  end

  methods (Access = protected)
    function [c,dc] = constraintEval(obj,x)
      c = double(subs(obj.p,obj.p_x,x));
      if nargout>1
        dc = double(subs(obj.dp,obj.p_x,x));
      end
    end
  end
end
