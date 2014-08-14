classdef ConstantPower < drakeFunction.DrakeFunction
  properties (SetAccess = immutable)
    power
  end
  methods
    function obj = ConstantPower(input_frame,output_frame,power)
      valuecheck(input_frame.dim,output_frame.dim);
      if ~isscalar(power)
        sizecheck(power,[input_frame.dim,1]);
      end
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.power = power;
    end

    function [f,df] = eval(obj,x)
      f = x.^obj.power;
      df = diag(obj.power.*x.^(obj.power-1));
    end
  end
end
