classdef LinearCombination < drakeFunction.DrakeFunction
  % DrakeFunction which given n points in the same frame, as well as n weights,
  % returns the linear combination of those points with those weights.
  %
  % \f[
  % f(r_1, \dots, r_n, \lambda) = \sum_{i=1}^n \lambda_i r_i,
  % \f]
  %
  % where
  %
  % \f[
  % r_i \in X,\; i=1,\dots,n
  % \f]
  %
  % and
  %
  % \f[
  % \lambda \in R^n.
  % \f]

  properties (SetAccess = immutable)
    n_pts     % Integer number of points
    dim_pts   % Integer dimension of the frame to which the points belong
    index_map               % Cell array of indices into the input vector
  end

  methods
    function obj = LinearCombination(n_pts,dim_pts)
      % obj = LinearCombination(n_pts,frame) returns a DrakeFunction object
      %   representing the linear combination of n_pts points in 'frame'.
      %
      %   @param n_pts  -- Integer number of points
      %   @param frame  -- CoordinateFrame to which the points belong
      %
      %   @retval obj   -- drakeFunction.LinearCombination object
      dim_input = n_pts*dim_pts + n_pts;
      obj = obj@drakeFunction.DrakeFunction(dim_input,dim_pts);
      obj.dim_pts = dim_pts;
      obj.n_pts = n_pts;
    end

    function [f,df] = eval(obj,x)
      % [f,df] = eval(obj,x) returns the linear combination for the given
      % points and weights.
      %
      % @param obj  -- drakeFunction.LinearCombination object
      % @param x    -- Input vector
      %
      % @retval f   -- Output vector
      % @retval df  -- Jacobian of output vector
      pts = reshape(x(1:obj.n_pts*obj.dim_pts),obj.dim_pts,obj.n_pts);
      weights = reshape(x(obj.n_pts*obj.dim_pts+(1:obj.n_pts)),[],1);
      f = pts*weights;
      df = zeros(obj.dim_pts,obj.n_pts+obj.n_pts*obj.dim_pts);
      for i = 1:obj.n_pts
        df(:,(i-1)*obj.dim_pts+(1:obj.dim_pts)) = weights(i)*eye(obj.dim_pts);
      end
      df(:,obj.n_pts*obj.dim_pts+(1:obj.n_pts)) = pts;
    end
  end
end
