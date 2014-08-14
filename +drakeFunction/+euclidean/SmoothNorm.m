classdef SmoothNorm < drakeFunction.DrakeFunction
  properties (SetAccess = immutable)
    smoothing_factor
  end

  methods
    function obj = SmoothNorm(input_frame,smoothing_factor)
      output_frame = drakeFunction.frames.R(1);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.smoothing_factor = smoothing_factor;
    end

    function [a,da] = eval(obj,r)
      a = sqrt(r'*r + obj.smoothing_factor^2);
      da = r'./a;
    end
  end
end
