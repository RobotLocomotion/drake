classdef Zeros < drakeFunction.Linear
  methods
    function obj = Zeros(input_frame,output_frame)
      obj = obj@drakeFunction.Linear(input_frame,output_frame,zeros(output_frame.dim,input_frame.dim));
    end
  end
end
