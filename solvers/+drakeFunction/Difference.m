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
    function obj = Difference(frame,n)
      % obj = Difference(frame,n) returns a function representing the
      %   differences between consectutive elements of a set of n points
      %   in the CoordinateFrame 'frame'
      %
      % obj = Difference(frame) is the same, but with n = 2
      %
      % @param frame  -- CoordinateFrame
      % @param n      -- Number of elements. 
      %                  Optional @default = 2
      if nargin < 2, n = 2; end
      typecheck(frame,'CoordinateFrame');
      integervaluedcheck(n);
      assert(n>=2,'Drake:DrakeFunction:Difference:BadInput',...
        'The number of points to difference, n, must be at least 2');
      input_frame = MultiCoordinateFrame.constructFrame(repmat({frame},1,n));
      output_frame = MultiCoordinateFrame.constructFrame(repmat({frame},1,n-1));
      nx = frame.dim;
      A = kron(spdiags(ones(n-1,1),0,n-1,n),eye(nx)) + kron(spdiags(-ones(n-1,1),1,n-1,n),eye(nx));
      obj = obj@drakeFunction.Linear(input_frame,output_frame,A);
    end
  end
end
