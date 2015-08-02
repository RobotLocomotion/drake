classdef Zeros < drakeFunction.Linear
  % DrakeFunction representing the zero-transformation from one
  % CoordinateFrame to another.
  methods
    function obj = Zeros(dim_input, dim_output)
      if nargin < 2, dim_output = dim_input; end
      % obj = drakeFunction.Zeros(input_frame,output_frame) returns a
      % function between two frames whose output is always zero.
      obj = obj@drakeFunction.Linear(zeros(dim_input, dim_output));
    end
  end
end
