classdef ConstantPower < drakeFunction.DrakeFunction
  properties (SetAccess = immutable)
    power   % Numeric vector or scalar
  end
  methods
    function obj = ConstantPower(varargin)
      % obj = ConstantPower(input_frame,output_frame,power) returns a
      %   DrakeFunction that raises its input elementwise to the given
      %   power
      %
      % obj = ConstantPower(frame,value) is the same, but with
      %   identical input and output frames
      %
      % @param input_frame    -- CoordinateFrame of the input
      % @param output_frame   -- CoordinateFrame of the output. Must
      %                          have the same dimension as input_frame
      % @param power          -- Numeric scalar or vector. The output of
      %                          the returned function is x.^power
      %
      % @retval fcn           -- drakeFunction.Linear object
      input_frame = varargin{1};
      if nargin > 2
        output_frame = varargin{2};
        power = varargin{3};
      else
        output_frame = input_frame;
        power = varargin{2};
      end
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
