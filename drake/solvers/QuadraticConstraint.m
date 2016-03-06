classdef QuadraticConstraint < Constraint
  % a quadratic constraint of the form lb <= .5 * x'*Q*x + b'*x <= ub
  % @param Q    -- A square matrix of size nx x nx
  % @param b    -- A double vector of size nx x 1
  properties(SetAccess = protected)
    Q
    b
  end

  methods
    function obj = QuadraticConstraint(lb,ub,Q,b)
      sizecheck(lb,[1,1]);
      sizecheck(ub,[1,1]);
      nx = size(Q,1);
      obj = obj@Constraint(lb,ub,nx,2);
      sizecheck(Q,[nx,nx]);
      sizecheck(b,[nx,1]);
      obj.Q = (Q + Q')/2;  % ensure that Q is symmetric
      obj.b = b;
    end
  end

  methods (Access = protected)
    function [c,dc,ddc] = constraintEval(obj,x)
      c = .5*x'*obj.Q*x + obj.b'*x;
      if nargout > 1
        dc = x'*obj.Q + obj.b';
      end
      if nargout > 2
        ddc = reshape(obj.Q,1,[]);
      end
    end
  end
end
