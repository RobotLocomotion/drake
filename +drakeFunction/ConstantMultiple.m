classdef ConstantMultiple < drakeFunction.Linear
  methods
    function obj = ConstantMultiple(varargin)
      % obj = ConstantMultiple(input_frame,output_frame,value) returns a
      %   function that multiplies its input elementwise by the given value
      %
      % obj = ConstantMultiple(frame,value) is the same, but with identical
      % input and output frames
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
