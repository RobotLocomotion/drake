classdef FunctionHandleConstraint < DifferentiableConstraint
  %FUNCTIONHANDLECONSTRAINT 
  % A DifferentiableConstraint implementation where the constraint is given
  % by a function handle.
  
  properties(SetAccess = protected)
    eval_handle
  end
  
  methods
    function obj = FunctionHandleConstraint(lb,ub,xdim,eval_handle)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param xdim  -- An int scalar. x is double vector of xdim x 1
      % @param eval_handle -- A function handle which performs the
      % constraint evaluation
      obj = obj@DifferentiableConstraint(lb,ub,xdim);
      obj.eval_handle = eval_handle;
    end
    
  end
  methods (Access = protected)
    function varargout = constraintEval(obj,varargin)
      if nargout <= 1,
        varargout{1} = obj.eval_handle(varargin{:});
      elseif nargout == 2,
        [varargout{1},varargout{2}] = obj.eval_handle(varargin{:});
      elseif nargout == 3,
        [varargout{1},varargout{2},varargout{3}] = obj.eval_handle(varargin{:});
      else
        error('Drake:FunctionHandleConstraint:UnsupportedEval','Only support 2nd order gradient at most');
      end      
    end
  end
end

