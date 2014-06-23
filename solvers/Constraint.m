classdef Constraint
  % Constraint that will be used for Drake solvers.
  % This is an abstract class, subclasses must implement the protected
  % method constraintEval. The eval function here is maintained to allow
  % use with geval, if the user does not specify required gradients.
  %
  % Constraints may be functions of multiple arguments.
  %
  % @param lb    -- The lower bound of the constraint
  % @param ub    -- The upper bound of the constraint
  % @param name  -- The name of the constraint. If not specified, it is an empty string
  % @param grad_method   -- A string indicating the method to compute gradient. Refer to
  % @param num_cnstr      -- An int scalar. The number of constraints
  % @param ceq_idx        -- The row index of the equality constraint
  % @param cin_idx        -- The row index of the inequality constraint
  % 'geval' for all supported method. Default is 'user'
  properties(SetAccess = protected)
    lb
    ub
    name
    num_cnstr
    ceq_idx
    cin_idx
  end
  
  properties
    grad_method
  end
  
  methods
    function obj = Constraint(lb,ub)
      % Constraint(lb,ub) or Constraint(lb,ub,eval_handle)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      if(~isnumeric(lb) || ~isnumeric(ub))
        error('Drake:Constraint:BadInputs','lb and ub should be numeric')
      end
      if(obj.num_cnstr ~= numel(obj.ub))
        error('Drake:Constraint:BadInputs','Constraint lb and ub should have same number of elements');
      end
      if(any(obj.lb>obj.ub))
        error('Drake:Constraint:BadInputs','Constraint lb should be no larger than ub');
      end
      obj.lb = lb(:);
      obj.ub = ub(:);
      obj.num_cnstr = numel(obj.lb);
      obj.name = repmat({''},obj.num_cnstr,1);
      
      c_idx = (1:obj.num_cnstr)';
      obj.ceq_idx = c_idx(obj.lb == obj.ub);
      obj.cin_idx = c_idx(obj.lb ~= obj.ub);
      
      obj.grad_method = 'user';
    end
    
    function varargout = eval(obj,varargin)
      % special casing 'user' to avoid geval for speed reasons
      if strcmp(obj.grad_method,'user')
        if nargout <= 1
          varargout{1} = obj.constraintEval(varargin{:});
        elseif nargout == 2
          [varargout{1},varargout{2}] = obj.constraintEval(varargin{:});
          
        elseif nargout == 3
          [varargout{1},varargout{2},varargout{3}] = obj.constraintEval(varargin{:});
          
        else
          error('Drake:Constraint:UnsupportedEval','Only support 2nd order gradient at most');
        end
      else
        % evaluate the constraint. Can be overloaded
        if(nargout <= 1)
          c = feval(@obj.constraintEval,varargin{:});
          varargout{1} = c;
        elseif(nargout == 2)
          [c,dc] = geval(@obj.constraintEval,varargin{:},struct('grad_method',obj.grad_method));
          varargout{1} = c;
          varargout{2} = dc;
        elseif(nargout == 3)
          [c,dc,ddc] = geval(@obj.constraintEval,varargin{:},struct('grad_method',obj.grad_method));
          varargout{1} = c;
          varargout{2} = dc;
          varargout{3} = ddc;
        else
          error('Drake:Constraint:UnsupportedEval','Only support 2nd order gradient at most');
        end
      end
    end
    
    function obj = setName(obj,name)
      % set the name of the constraint
      obj.name = name;
    end
  end
  
  methods(Abstract, Access = protected)
    varargout = constraintEval(obj, varargin);
  end
  
end
