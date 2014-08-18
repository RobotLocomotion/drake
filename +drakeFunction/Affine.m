classdef Affine < drakeFunction.DrakeFunction
  % DrakeFunction representing an affine map:
  % \f[
  % f(x) = Ax + b
  % \f]
  properties (SetAccess = immutable)
    A
    b
  end
  methods
    function obj = Affine(input_frame,output_frame,A,b)
      % obj = drakeFunction.Affine(input_frame,output_frame,A,b) returns
      %   an affine function from input_frame to output_frame
      % @param input_frame    -- CoordinateFrame representing the domain
      % @param output_frame   -- CoordinateFrame representing the range
      % @param A              -- [n x m] matrix where m and n are the
      %                          dimension of input_frame and
      %                          output_frame respectively
      % @param b              -- n-element column vector
      %
      % @retval obj           -- drakeFunction.Affine object
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

    function [iCfun, jCvar] = getSparsityPattern(obj)
      [iCfun, jCvar] = find(obj.A);
    end
  end
end
