classdef Linear < drakeFunction.Affine
  % DrakeFunction of the form:
  %
  % \f[
  % f(x) = Ax
  % \f]
  methods
    function obj = Linear(A)
      % obj = Linear(A) returns a
      % drakeFunction.Linear object.
      %
      % @param A            -- Numeric matrix. The output of the
      %                        returned function is A*x.
      b = zeros(size(A,1), 1);
      obj = obj@drakeFunction.Affine(A,b);
    end
  end
end
