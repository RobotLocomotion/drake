classdef ConstantMultiple < drakeFunction.Linear
  methods
    function obj = ConstantMultiple(varargin)
      % obj = ConstantMultiple(input_frame,output_frame,value) returns a
      %   drakeFunction.Linear that multiplies its input elementwise by
      %   the given value. 
      %
      % obj = ConstantMultiple(frame,value) is the same, but with
      %   identical input and output frames
      % 
      % @param input_frame    -- CoordinateFrame of the input
      % @param output_frame   -- CoordinateFrame of the output. Must have
      %                          the same dimension as input_frame
      % @param value          -- Numeric scalar or vector. The output of the
      %                          returned function is value.*x
      %
      % @retval obj           -- drakeFunction.Linear object
      input_frame = varargin{1};
      if nargin > 2
        output_frame = varargin{2};
        value = varargin{3};
      else
        output_frame = input_frame;
        value = varargin{2};
      end
      if isscalar(value)
        A = value*eye(input_frame.dim);
      else
        A = diag(value);
      end
      obj = obj@drakeFunction.Linear(input_frame,output_frame,A);
    end
  end
end
