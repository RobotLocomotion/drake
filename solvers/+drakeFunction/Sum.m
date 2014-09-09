classdef Sum < drakeFunction.Linear
  % Sum of N vectors in the same frame 
  methods
    function obj = Sum(frame,N)
      % obj = drakeFunction.Sum(frame,N) constructs a Sum object
      %
      % @param frame  -- CoordinateFrame to which the sum and each of the
      %                  terms belong
      % @param N      -- Integer number of terms

      integervaluedcheck(N);

      input_frame = MultiCoordinateFrame(repmat({frame},1,N));
      output_frame = frame;

      A = repmat(eye(frame.dim),1,N);

      obj = obj@drakeFunction.Linear(input_frame,output_frame,A);
    end
  end
end
