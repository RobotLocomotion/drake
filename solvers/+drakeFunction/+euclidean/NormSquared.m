classdef NormSquared < drakeFunction.DrakeFunction
  % DrakeFunction representing the square of the Euclidean norm (or weighted
  % Euclidean norm) for points in a given frame.
  properties
    Q           % Weighting matrix
  end

  methods
    function obj = NormSquared(input_frame,Q)
      % obj = NormSquared(input_frame,Q) returns a NormSquared object
      %
      % @param input_frame  -- CoordinateFrame to which the input belongs
      % @param Q            -- Numerical weighting matrix. Optional
      %                        @default eye(input_frame.dim)
      %
      % @retval obj         -- drakeFunction.euclidean.NormSquared object
      n = input_frame.dim;
      if nargin < 2, 
        Q = []; 
      else
        sizecheck(Q,[n,n]);
        if any(eig(Q) <= 0)
          error('Drake:drakeFunction:euclidean:NormSquared:NonPositiveDefiniteQ', ...
            'The weighting matrix, Q, should be positive definite');
        end
        if all(all(Q == eye(size(Q)))), Q = []; end % Un-weighted case
      end
      output_frame = drakeFunction.frames.realCoordinateSpace(1);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.Q = Q;
      obj.is_weighted = ~isempty(Q);
    end

    function [a,da] = eval(obj,r)
      if obj.is_weighted
        a = r'*obj.Q*r;
        da = 2*r'*obj.Q;
      else
        a = r'*r;
        da = 2*r';
      end
    end
  end

  properties (Access = private)
    is_weighted % Logical scalar 
  end
end
