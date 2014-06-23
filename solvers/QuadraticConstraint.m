classdef QuadraticConstraint < DifferentiableConstraint
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
      obj = obj@DifferentiableConstraint(lb,ub,nx);
      sizecheck(Q,[nx,nx]);
      sizecheck(b,[nx,1]);
      obj.Q = Q;
      obj.b = b;
    end
  end
  
  methods (Access = protected)
    function [c,dc] = constraintEval(obj,x)
      c = (x-obj.a)'*obj.Q*(x-obj.a);
      dc = 2*(x-obj.a)'*obj.Q;
    end
  end
end