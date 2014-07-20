classdef FunctionHandleConstraint < Constraint
  %FUNCTIONHANDLECONSTRAINT
  % A Constraint implementation where the constraint is given
  % by a function handle.

  properties(SetAccess = protected)
    eval_handle
  end

  methods
    function obj = FunctionHandleConstraint(lb,ub,xdim,eval_handle,grad_level)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param xdim  -- An int scalar. x is double vector of xdim x 1
      % @param eval_handle -- A function handle which performs the
      % constraint evaluation
      % @param grad_level optional user_gradient level.  see Constraint constructor. @default -1

      if nargin<5, grad_level = -1; end

      obj = obj@Constraint(lb,ub,xdim,grad_level);
      obj.eval_handle = eval_handle;
    end

    function obj = setEvalHandle(obj,eval_handle_new)
      obj.eval_handle = eval_handle_new;
    end

  end
  methods (Access = protected)
    function varargout = constraintEval(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = obj.eval_handle(varargin{:});
    end
  end
end
