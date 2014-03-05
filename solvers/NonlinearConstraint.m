classdef NonlinearConstraint < Constraint
  % Nonlinear constraint. Support computing the non-zero entries in the first order
  % gradient. The default sparsity pattern is the the gradient matrix being dense. Use
  % 'setSparsityStructure' to set the sparsity pattern.
  % @param num_cnstr      -- An int scalar. The number of constraints
  % @param xdim           -- An int scalar. The decision variable is an xdim x 1 double
  % vector
  % @param iCfun          -- An int vector. The row index of non-zero entries of the
  % gradient matrix
  % @param jCvar          -- An int vector. The column index of the non-zero entries of
  % the gradient matrix
  % @param nnz            -- An int scalar. The maximal number of non-zero entries in the
  % gradient matrix
  % @param ceq_idx        -- The row index of the equality constraint
  % @param cin_idx        -- The row index of the inequality constraint
  properties(SetAccess = protected)
    num_cnstr
    xdim
    iCfun
    jCvar
    nnz
    ceq_idx
    cin_idx
  end
  
  methods
    function obj = NonlinearConstraint(c_lb,c_ub,x_lb,x_ub,eval_handle)
      % @param c_lb    -- The lower bound of the constraint
      % @param c_ub    -- The upper bound of the constraint
      % @param x_lb    -- The lower bound of the decision variables
      % @param x_ub    -- The upper bound of the decision variables
      % @param eval_handle   -- The function handle to evaluate constraint. An optional
      % argument.
      if(nargin < 5)
        eval_handle = [];
      end
      obj = obj@Constraint(c_lb,c_ub,x_lb,x_ub,eval_handle);
      if(~isnumeric(c_lb) || ~isnumeric(c_ub))
        error('Drake:NonlinearConstraint:BadInputs','NonlinearConstraint c_lb and c_ub should be numeric')
      end
      obj.c_lb = obj.c_lb(:);
      obj.c_ub = obj.c_ub(:);
      obj.num_cnstr = numel(obj.c_lb);
      if(obj.num_cnstr ~= numel(obj.c_ub))
        error('Drake:NonlinearConstraint:BadInputs','NonlinearConstraint c_lb and c_ub should have same number of elements');
      end
      if(any(obj.c_lb>obj.c_ub))
        error('Drake:NonlinearConstraint:BadInputs','NonlinearConstraint c_lb should be no larger than c_ub');
      end
      c_idx = (1:obj.num_cnstr)';
      obj.ceq_idx = c_idx(obj.relation == Constraint.equal);
      obj.cin_idx = c_idx(obj.relation == Constraint.ineq);
      if(~isnumeric(obj.x_lb) || ~isnumeric(obj.x_ub))
        error('Drake:NonlinearConstraint:BadInputs','NonlinearConstraint x_lb and x_ub should be numeric');
      end
      obj.xdim = numel(obj.x_lb);
      obj.x_lb = obj.x_lb(:);
      obj.x_ub = obj.x_ub(:);
      if(obj.xdim ~= numel(obj.x_ub))
        error('Drake:NonlinearConstraint:BadInputs','NonlinearConstraint x_lb and x_ub should have the same number of elements');
      end
      if(any(obj.x_lb>obj.x_ub))
        error('Drake:NonlinearConstraint:BadInputs','NonlinearConstraint x_lb shoulde be no larger than x_ub');
      end
      obj.iCfun = reshape(bsxfun(@times,(1:obj.num_cnstr)',ones(1,obj.xdim)),[],1);
      obj.jCvar = reshape(bsxfun(@times,1:obj.xdim,ones(obj.num_cnstr,1)),[],1);
      obj.nnz = obj.num_cnstr*obj.xdim;
    end
    
    function obj = setSparseStructure(obj,iCfun,jCvar,nnz)
      % set the sparse structure of the 1st order gradient matrix
      % @param iCfun          -- An int vector. The row index of non-zero entries of the
      % gradient matrix
      % @param jCvar          -- An int vector. The column index of the non-zero entries of
      % the gradient matrix
      % @param nnz            -- An int scalar. The number of non-zero entries in the
      % gradient matrix
      iCfun = iCfun(:);
      jCvar = jCvar(:);
      if(numel(iCfun) ~= numel(jCvar) || numel(iCfun) ~= nnz)
        error('Drake:NonlinearConstraint:WrongSparseStructure','NonlinearConstraint iCfun and jCvar should have the same number of elements');
      end
      if(any(iCfun<1) || any(iCfun>obj.num_cnstr) || any(jCvar<1) || any(jCvar>obj.xdim))
        error('Drake:NonlinearConstraint:WrongSparseStructure','NonlinearConstraint iCfun or jCvar has incorrect index');
      end
      obj.iCfun = iCfun;
      obj.jCvar = jCvar;
      obj.nnz = nnz;
    end
    
    function [c,dc_val] = evalSparse(obj,varargin)
      % return the constraint value and the non-zero entries of its first order gradient
      % @retval c      -- An obj.num_cnstr x 1 double vector. The constraint value
      % @retval dc_val -- A column vector. The non-zero values of the 1st order gradient.
      [c,dc] = obj.eval(varargin{:});
      dc_val = dc(sub2ind([obj.num_cnstr,obj.xdim],obj.iCfun,obj.jCvar));
    end
  end
end