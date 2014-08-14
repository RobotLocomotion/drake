classdef Composed < drakeFunction.DrakeFunction
  % Composed    DrakeFunction representing the composition of two functions
  % Implements
  %   f = fcn_outer(fcn_inner(x))
  properties (SetAccess = immutable)
    fcn_outer   % DrakeFunction representing the outer function 
    fcn_inner   % DrakeFunction representing the inner function 
  end
  methods 
    function obj = Composed(fcn_outer,fcn_inner)
      % obj = drakeFunction.Composed(fcn_outer,fcn_inner) returns a
      % Composed object representing the composition of the given
      % functions
      %
      % @param fcn_outer    -- Any DrakeFunction
      % @param fcn_inner    -- Any DrakeFunction
      %
      % @retval obj         -- Composed function
      typecheck(fcn_outer,'drakeFunction.DrakeFunction');
      typecheck(fcn_inner,'drakeFunction.DrakeFunction');
      assert(isequal_modulo_transforms(fcn_outer.input_frame,fcn_inner.output_frame));
      obj = obj@drakeFunction.DrakeFunction(fcn_inner.getInputFrame(),...
                                   fcn_outer.getOutputFrame());
      obj.fcn_outer = fcn_outer;
      obj.fcn_inner = fcn_inner;
    end

    function [f,df] = eval(obj,x)
      [f_inner, df_inner] = obj.fcn_inner.eval(x);
      [f,df_df_inner] = obj.fcn_outer.eval(f_inner);
      df = df_df_inner*df_inner;
    end
  end
end
