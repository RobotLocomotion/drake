classdef Constraint
  % Constraint that will be used for Drake solvers
  % @param lb    -- The lower bound of the constraint
  % @param ub    -- The upper bound of the constraint
  % @param name  -- The name of the constraint. If not specified, it is an empty string
  % @param grad_method   -- A string indicating the method to compute gradient. Refer to
  % 'geval' for all supported method. Default is 'user'
  properties(SetAccess = protected)
    lb
    ub
    name
  end
  
  properties
    grad_method
  end
  
  properties(SetAccess = protected,GetAccess = protected)
    eval_handle   % Function handle to evaluate constraint.
  end
  
  methods
    function obj = Constraint(lb,ub,eval_handle)
      % Constraint(lb,ub) or Constraint(lb,ub,eval_handle)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param eval_handle   -- The function handle to evaluate constraint. An optional
      % argument.
      if(nargin<3)
        eval_handle = [];
      end
      obj.lb = lb;
      obj.ub = ub;
      obj.eval_handle = eval_handle;
      obj.name = {''};
      obj.grad_method = 'user';
    end
    
    function varargout = eval(obj,varargin)
      % evaluate the constraint. Can be overloaded
      if(nargout == 1)
        c = feval(obj.eval_handle,varargin{:});
        varargout{1} = c;
      elseif(nargout == 2)
        [c,dc] = geval(obj.eval_handle,varargin{:},struct('grad_method',obj.grad_method));
        varargout{1} = c;
        varargout{2} = dc;
      elseif(nargout == 3)
        [c,dc,ddc] = geval(obj.eval_handle,varargin{:},struct('grad_method',obj.grad_method));
        varargout{1} = c;
        varargout{2} = dc;
        varargout{3} = ddc;
      else
        error('Drake:Constraint:UnsupportedEval','Only support 2nd order gradient at most');
      end
    end
       
    function obj = setName(obj,name)
      % set the name of the constraint
      obj.name = name;
    end
  end
  
end