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

    function obj_new = setEvalHandle(obj,eval_handle_new)
      % obj_new = setEvalHandle(obj,eval_handle_new) returns a
      % FunctionHandleConstraint object the same as this one, but with a new
      % eval_handle. 
      %
      % Note: We're constructing a new constraint object here because altering
      % the eval_handle property of an existing constraint object makes
      % concatenating that object in a cell array *very* slow. We don't know why
      % that is.
      
      obj_new = FunctionHandleConstraint(obj.lb,obj.ub,obj.xdim,eval_handle_new);
      obj_new = obj_new.setSparseStructure(obj.iCfun,obj.jCvar);
      obj_new = obj_new.setName(obj.name);
    end

  end
  methods (Access = protected)
    function varargout = constraintEval(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = obj.eval_handle(varargin{:});
    end
  end
end
