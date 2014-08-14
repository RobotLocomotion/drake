classdef DrakeFunctionConstraint < Constraint
  properties (SetAccess = private)
    fcn
  end
  methods
    function obj = DrakeFunctionConstraint(lb,ub,fcn)
      sizecheck(lb,[fcn.getOutputFrame().dim,1]);
      sizecheck(ub,[fcn.getOutputFrame().dim,1]);
      obj = obj@Constraint(lb,ub,fcn.getInputFrame().dim,1);
      obj.fcn = fcn;
    end
  end

  methods (Access = protected)
    function varargout = constraintEval(obj,x)
      varargout = cell(nargout,1);
      [varargout{:}] = obj.fcn.eval(x);
    end
  end
end
