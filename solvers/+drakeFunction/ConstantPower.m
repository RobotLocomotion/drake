classdef ConstantPower < drakeFunction.DrakeFunction
  properties (SetAccess = immutable)
    power   % Numeric vector or scalar
  end
  methods
    function obj = ConstantPower(dim_input, power)
      % obj = ConstantPower(dim_input, power) returns a
      %   DrakeFunction that raises its input elementwise to the given
      %   power
      %
      % @param dim_input        -- Length of the input vector
      % @param power          -- Numeric scalar or vector. The output of
      %                          the returned function is x.^power
      %
      % @retval obj           -- drakeFunction.Linear object
      if ~isscalar(power)
        sizecheck(power,[dim_input,1]);
      end
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_input);
      obj.power = power;
    end

    function [f,df] = eval(obj,x)
      f = x.^obj.power;
      df = diag(obj.power.*x.^(obj.power-1));
    end
  end
end
