classdef Difference < drakeFunction.Linear
  % DrakeFunction representing the first difference between a series of points:
  %
  % \f[
  % f(x) =
  % \begin{bmatrix}
  % x_2 - x_1     \\
  % x_3 - x_2     \\
  % \vdots        \\
  % x_n - x_{n-1}
  % \end{bmatrix}
  % \f]
  methods
    function obj = Difference(m,n)
      % obj = Difference(frame,n) returns a function representing the
      %   differences between consectutive elements of a set of n points
      %   in R^m
      %
      % obj = Difference(frame) is the same, but with n = 2
      %
      % @param m      -- length of each element
      % @param n      -- Number of elements. 
      %                  Optional @default = 2
      if nargin < 2, n = 2; end
      integervaluedcheck(m);
      integervaluedcheck(n);
      assert(n>=2,'Drake:DrakeFunction:Difference:BadInput',...
        'The number of points to difference, n, must be at least 2');
      A = kron(spdiags(ones(n-1,1),0,n-1,n),eye(m)) + kron(spdiags(-ones(n-1,1),1,n-1,n),eye(m));
      obj = obj@drakeFunction.Linear(A);
    end
  end
end
