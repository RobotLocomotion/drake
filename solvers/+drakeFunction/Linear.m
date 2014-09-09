classdef Linear < drakeFunction.Affine
  % DrakeFunction of the form:
  %
  % \f[
  % f(x) = Ax
  % \f]
  methods
    function obj = Linear(input_frame,output_frame,A)
      % obj = Linear(input_frame,output_frame,A) returns a
      % drakeFunction.Linear object.
      %
      % @param input_frame  -- CoordinateFrame object
      % @param output_frame -- CoordinateFrame object
      % @param A            -- Numeric matrix. The output of the
      %                        returned function is A*x.
      b = zeros(output_frame.dim,1);
      obj = obj@drakeFunction.Affine(input_frame,output_frame,A,b);
    end
  end
end
