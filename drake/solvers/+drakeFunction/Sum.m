classdef Sum < drakeFunction.Linear
  % Sum of N vectors of size M
  methods
    function obj = Sum(M, N)
      % obj = drakeFunction.Sum(frame,N) constructs a Sum object
      %
      % @param frame  -- CoordinateFrame to which the sum and each of the
      %                  terms belong
      % @param N      -- Integer number of terms

      integervaluedcheck(N);
      integervaluedcheck(M);

      A = repmat(eye(M),1,N);

      obj = obj@drakeFunction.Linear(A);
    end
  end
end
