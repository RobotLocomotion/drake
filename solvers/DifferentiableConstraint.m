classdef DifferentiableConstraint < Constraint
  % Differentiable constraint. Support computing the non-zero entries in the first order
  % gradient. The default sparsity pattern is the the gradient matrix being dense. Use
  % 'setSparsityStructure' to set the sparsity pattern.
  %
  % Extends Constraint by identifying the sparsity structure of the
  % gradient, but is still an abstract class. In the case where the
  % constraint is a function of multiple arguments, the gradient
  % information here (and xdim) considers the arguments in a stacked
  % manner, [arg1(:),arg2(:);...]
  %
  % @param xdim           -- An int scalar. The decision variable is an xdim x 1 double
  % vector
  % @param iCfun          -- An int vector. The row index of non-zero entries of the
  % gradient matrix
  % @param jCvar          -- An int vector. The column index of the non-zero entries of
  % the gradient matrix
  % @param nnz            -- An int scalar. The maximal number of non-zero entries in the
  % gradient matrix
  properties(SetAccess = protected)
    xdim
    iCfun
    jCvar
    nnz
  end
  
  methods
    function obj = DifferentiableConstraint(lb,ub,xdim)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param xdim  -- An int scalar. x is double vector of xdim x 1
      obj = obj@Constraint(lb,ub);
      if(~isnumeric(xdim) || numel(xdim) ~= 1 || xdim<0 || xdim ~= floor(xdim))
        error('Drake:DifferentiableConstraint:BadInputs','xdim should be a non-negative integer');
      end
      obj.xdim = xdim;
      
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
        error('Drake:DifferentiableConstraint:WrongSparseStructure','iCfun and jCvar should have the same number of elements');
      end
      if(any(iCfun<1) || any(iCfun>obj.num_cnstr) || any(jCvar<1) || any(jCvar>obj.xdim))
        error('Drake:DifferentiableConstraint:WrongSparseStructure','iCfun or jCvar has incorrect index');
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
        error('Drake:DifferentiableConstraint:name should be a cell array of string');
      end
      sizecheck(name,[obj.num_cnstr,1]);
      obj.name = name;
    end
  end
end