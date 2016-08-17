classdef SmoothNorm < drakeFunction.DrakeFunction
  % DrakeFunction representing the smooth norm of a vector:
  %
  % \f[
  % f(r) = \sqrt{r^\prime r + \alpha^2}
  % \f]
  %
  % where \f$\alpha\f$ is a smoothing factor.
  properties (SetAccess = immutable)
    smoothing_factor  % Scalar smoothing factor
  end

  methods
    function obj = SmoothNorm(dim_input,smoothing_factor)
      % obj = SmoothNorm(input_frame,smoothing_factor)
      % 
      % @param dim_input          -- Length of the input vector
      % @param smoothing_factor   -- Numerical scalar
      %
      % @retval obj               -- drakeFunction.euclidean.SmoothNorm
      %                              object
      sizecheck(smoothing_factor,[1,1]);
      typecheck(smoothing_factor,'numeric');
      obj = obj@drakeFunction.DrakeFunction(dim_input,1);
      obj.smoothing_factor = smoothing_factor;
    end

    function [a,da] = eval(obj,r)
      a = sqrt(r'*r + obj.smoothing_factor^2);
      da = r'./a;
    end
  end
end
