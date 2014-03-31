classdef QuadraticConstraint < NonlinearConstraint
  % lb <= (x-a)'Q(x-a) <= ub
  % @param Q    -- A square matrix of size nx x nx
  % @param a    -- A double vector of size nx x 1
  properties(SetAccess = protected)
    Q
    a
  end
  
  methods
    function obj = QuadraticConstraint(lb,ub,Q,a)
      sizecheck(lb,[1,1]);
      sizecheck(ub,[1,1]);
      nx = size(Q,1);
      obj = obj@NonlinearConstraint(lb,ub,nx);
      sizecheck(Q,[nx,nx]);
      sizecheck(a,[nx,1]);
      obj.Q = Q;
      obj.a = a;
    end
    
    function [c,dc] = eval(obj,x)
      c = (x-obj.a)'*obj.Q*(x-obj.a);
      dc = 2*(x-obj.a)'*obj.Q;
    end
  end
end