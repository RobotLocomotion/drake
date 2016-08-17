classdef DrakeFunctionConstraint < Constraint
  properties (SetAccess = private)
    fcn
  end
  methods
    function obj = DrakeFunctionConstraint(lb,ub,fcn)
      sizecheck(lb,[fcn.dim_output,1]);
      sizecheck(ub,[fcn.dim_output,1]);
      obj = obj@Constraint(lb,ub,fcn.dim_input,1);
      obj.fcn = fcn;
      [iCfun,jCvar] = getSparsityPattern(obj.fcn);
      obj = setSparseStructure(obj,iCfun,jCvar);
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
      obj = setSparseStructure@Constraint(obj,iCfun,jCvar);
    end
  end

  methods (Access = protected)
    function varargout = constraintEval(obj,x)
      varargout = cell(nargout,1);
      [varargout{:}] = eval(obj.fcn,x);
    end
  end
end
