classdef PeriodicConstraint < LinearConstraint
  % enforce periodicity between all the states whithin
  % the specified tolerance   

  methods
    function obj = PeriodicConstraint(tol)
      lb = -abs(tol);
      ub = abs(tol);
      A = [speye(numel(tol)),-speye(numel(tol))];
      obj = obj@LinearConstraint(lb,ub,A);
    end
  end
end