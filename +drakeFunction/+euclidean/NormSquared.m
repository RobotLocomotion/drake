classdef NormSquared < drakeFunction.DrakeFunction
  properties
    Q
    is_weighted
  end
  methods
    function obj = NormSquared(input_frame,Q)
      n = input_frame.dim;
      if nargin < 2, 
        Q = []; 
      else
        sizecheck(Q,[n,n]);
      end
      output_frame = drakeFunction.frames.R(1);
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
end
