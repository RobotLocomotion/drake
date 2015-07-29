classdef ConstantMultiple < drakeFunction.Linear
  methods
    function obj = ConstantMultiple(dim_input, value)
      % obj = ConstantMultiple(dim_input, value) returns a
      %   drakeFunction.Linear that multiplies its input elementwise by
      %   the given value. 
      % 
      % @param dim_input        -- Length of the input vector
      % @param value          -- Numeric scalar or vector. The output of the
      %                          returned function is value.*x
      %
      % @retval obj           -- drakeFunction.Linear object
      if isscalar(value)
        A = value*eye(dim_input);
      else
        A = diag(value);
      end
      obj = obj@drakeFunction.Linear(A);
    end
  end
end
