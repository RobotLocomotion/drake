classdef Constraint
  % Constraint that will be used for Drake solvers.
  % This is an abstract class, subclasses must implement the protected
  % method constraintEval. The eval function here is maintained to allow
  % use with geval, if the user does not specify required gradients.
  %
  % Constraints may be functions of multiple arguments.
  %

  properties(SetAccess = protected)
    lb      % The lower bound of the constraint
    ub      % The upper bound of the constraint
    xdim    % The name of the constraint. If not specified, it is an empty string
    num_cnstr % An int scalar. The number of constraints
    name    % cell array of constraint names
    ceq_idx   % The row index of the equality constraint
    cin_idx   % The row index of the inequality constraint

    % gradient sparsity information
    iCfun   % An int vector. The row index of non-zero entries of the gradient matrix
    jCvar   % An int vector. The column index of the non-zero entries of the gradient matrix
    nnz     % An int scalar. The maximal number of non-zero entries in the gradient matrix
  end

  properties
    grad_level    % derivative level of user gradients provided
    grad_method   % A string indicating the method to compute gradient. Refer to
                  %    'geval' for all supported method. @default 'user'
  end

  methods
    function obj = Constraint(lb,ub,xdim,grad_level)
      % Constraint(lb,ub) or Constraint(lb,ub,eval_handle)
      % @param lb    The lower bound of the constraint
      % @param ub    The upper bound of the constraint
      % @param xdim  size of the input
      % @param grad_level derivative level of user gradients
      %               -2 - non-differentiable
      %               -1 - unknown
      %                0 - no user gradients
      %                1 - first derivatives provided
      %                ...
      %              @default -1
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

      if(~isnumeric(xdim) || numel(xdim) ~= 1 || xdim<0 || xdim ~= floor(xdim))
        error('Drake:Constraint:BadInputs','xdim should be a non-negative integer');
      end
      obj.xdim = xdim;

      if (nargin<4), grad_level = -1; end
      obj.grad_level = grad_level;

      obj.num_cnstr = numel(obj.lb);
      obj.name = repmat({''},obj.num_cnstr,1);

      c_idx = (1:obj.num_cnstr)';
      obj.ceq_idx = c_idx(obj.lb == obj.ub);
      obj.cin_idx = c_idx(obj.lb ~= obj.ub);

      obj.grad_method = 'user';

      obj.iCfun = reshape(bsxfun(@times,(1:obj.num_cnstr)',ones(1,obj.xdim)),[],1);
      obj.jCvar = reshape(bsxfun(@times,1:obj.xdim,ones(obj.num_cnstr,1)),[],1);
      obj.nnz = obj.num_cnstr*obj.xdim;
    end

    function obj = setSparseStructure(obj,iCfun,jCvar)
      % set the sparse structure of the 1st order gradient matrix
      % @param iCfun          -- An int vector. The row index of non-zero entries of the
      % gradient matrix
      % @param jCvar          -- An int vector. The column index of the non-zero entries of
      % the gradient matrix
      iCfun = iCfun(:);
      jCvar = jCvar(:);
      if(numel(iCfun) ~= numel(jCvar))
        error('Drake:Constraint:WrongSparseStructure','iCfun and jCvar should have the same number of elements');
      end
      if(any(iCfun<1) || any(iCfun>obj.num_cnstr) || any(jCvar<1) || any(jCvar>obj.xdim))
        error('Drake:Constraint:WrongSparseStructure','iCfun or jCvar has incorrect index');
      end
      obj.iCfun = iCfun;
      obj.jCvar = jCvar;
      obj.nnz = numel(iCfun);
    end

    function checkGradient(obj,tol,varargin)
      % Check the accuracy and sparsity pattern of the gradient
      % @param tol    -- A double scalar. The tolerance of the user gradient, compared with
      % numerical gradient
      % @param varargin  -- The argument passed to eval function
      [~,dc] = obj.eval(varargin{:});
      [~,dc_numeric] = geval(@obj.eval,varargin{:},struct('grad_method','numerical'));
      valuecheck(dc,dc_numeric,tol);
      valuecheck(dc,sparse(obj.iCfun,obj.jCvar,dc(sub2ind([obj.num_cnstr,obj.xdim],obj.iCfun,obj.jCvar)),obj.num_cnstr,obj.xdim,obj.nnz));
    end

    function obj = setName(obj,name)
      % @param name   -- A cell array, name{i} is the name string of i'th constraint
      if(~iscellstr(name))
        error('Drake:Constraint:name should be a cell array of string');
      end
      sizecheck(name,[obj.num_cnstr,1]);
      obj.name = name;
    end

    function varargout = eval(obj,varargin)
      if obj.grad_level==-2  % no gradients available
        varargout{1} = obj.constraintEval(varargin{:});
      else
        % special casing 'user' to avoid geval for speed reasons
        varargout=cell(1,nargout);
        if (isempty(obj.grad_method) && nargout<=obj.grad_level+1) ...
            || all(strcmp('user',obj.grad_method))
          [varargout{:}] = obj.constraintEval(varargin{:});
        else
          gopt.grad_method = obj.grad_method;
          gopt.grad_level = obj.grad_level;
          [varargout{:}] = geval(@obj.constraintEval,varargin{:},gopt);
        end
      end
    end
  end

  methods(Abstract, Access = protected)
    varargout = constraintEval(obj, varargin);
  end

end
