classdef Constraint
  % Constraint that will be used for Drake solvers
  % @param lb    -- The lower bound of the constraint
  % @param ub    -- The upper bound of the constraint
  % @param relation   -- A vector of with the same size as lb or ub. The relationship of
  % the constraint. The supported relationships are in the 'Constant' properties
  properties(SetAccess = protected)
    c_lb
    c_ub
    x_lb
    x_ub
    relation
  end
  
  properties(SetAccess = protected,GetAccess = protected)
    eval_handle   % Function handle to evaluate constraint.
  end
  
  properties(Constant)
    equal = 1
    ineq = 2
  end
  
  methods
    function obj = Constraint(c_lb,c_ub,x_lb,x_ub,eval_handle)
      % Constraint(lb,ub) or Constraint(lb,ub,eval_handle)
      % @param c_lb    -- The lower bound of the constraint
      % @param c_ub    -- The upper bound of the constraint
      % @param x_lb    -- The lower bound of the decision variables
      % @param x_ub    -- The upper bound of the decision variables
      % @param eval_handle   -- The function handle to evaluate constraint. An optional
      % argument.
      if(nargin<5)
        eval_handle = [];
      end
      obj.c_lb = c_lb;
      obj.c_ub = c_ub;
      obj.x_lb = x_lb;
      obj.x_ub = x_ub;
      obj.relation = zeros(size(obj.c_lb));
      obj.relation(obj.c_lb<obj.c_ub) = Constraint.ineq;
      obj.relation(obj.c_lb == obj.c_ub) = Constraint.equal;
      obj.eval_handle = eval_handle;
    end
    
    function varargout = eval(obj,varargin)
      % evaluate the constraint. Can be overloaded
      if(nargout == 1)
        c = feval(obj.eval_handle,varargin{:});
        varargout{1} = c;
      elseif(nargout == 2)
        [c,dc] = geval(obj.eval_handle,varargin{:},struct('grad_method','user_then_numerical'));
        varargout{1} = c;
        varargout{2} = dc;
      elseif(nargout == 3)
        [c,dc,ddc] = geval(obj.eval_handle,varargin{:},struct('grad_method','user_then_numerical'));
        varargout{1} = c;
        varargout{2} = dc;
        varargout{3} = ddc;
      else
        error('Drake:Constraint:UnsupportedEval','Only support 2nd order gradient at most');
      end
    end
    
    
    
    function [c_lb,c_ub] = bounds(obj)
      % Return the bounds on the constraints
      c_lb = obj.c_lb;
      c_ub = obj.c_ub;
    end
  end
  
end