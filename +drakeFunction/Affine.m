classdef Affine < drakeFunction.DrakeFunction
  properties (SetAccess = immutable)
    A
    b
  end
  methods
    function obj = Affine(input_frame,output_frame,A,b)
      sizecheck(A,[output_frame.dim,input_frame.dim]);
      sizecheck(b,[output_frame.dim,1]);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.A = A;
      obj.b = b;
    end

    function [f,df] = eval(obj,x)
      f = obj.A*x + obj.b;
      df = obj.A;
    end
  end
end
