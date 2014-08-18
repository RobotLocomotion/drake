classdef Zeros < drakeFunction.Linear
  % DrakeFunction representing the zero-transformation from one
  % CoordinateFrame to another.
  methods
    function obj = Zeros(input_frame,output_frame)
      % obj = drakeFunction.Zeros(input_frame,output_frame) returns a
      % function between two frames whose output is always zero.
      obj = obj@drakeFunction.Linear(input_frame,output_frame,zeros(output_frame.dim,input_frame.dim));
    end
  end
end
