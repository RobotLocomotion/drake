classdef Linear < drakeFunction.Affine
  methods
    function obj = Linear(input_frame,output_frame,A)
      b = zeros(output_frame.dim,1);
      obj = obj@drakeFunction.Affine(input_frame,output_frame,A,b);
    end
  end
end
