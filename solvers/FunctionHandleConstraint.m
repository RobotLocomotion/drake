classdef FunctionHandleConstraint < Constraint
  %FUNCTIONHANDLECONSTRAINT 
  % A Constraint implementation where the constraint is given
  % by a function handle.
  
  properties(SetAccess = protected)
    eval_handle
  end
  
  methods
    function obj = FunctionHandleConstraint(lb,ub,eval_handle)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param eval_handle -- A function handle which performs the
      % constraint evaluation
      
      % NOTE: if you got a "too many inputs" error, it's probably because
      % the old FunctionHandleConstraint class is now
      % FunctionHandleDifferentiableConstraint
      
      obj = obj@Constraint(lb,ub);
      obj.eval_handle = eval_handle;
    end

    function obj = setEvalHandle(obj,eval_handle_new)
      obj.eval_handle = eval_handle_new;
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

