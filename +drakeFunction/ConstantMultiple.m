classdef ConstantMultiple < drakeFunction.Linear
  methods
    function obj = ConstantMultiple(input_frame,output_frame,value)
      if isscalar(value)
        A = value*eye(input_frame.dim);
      else
        A = diag(value);
      end
      obj = obj@drakeFunction.Linear(input_frame,output_frame,A);
    end
  end
end
