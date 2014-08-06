classdef LinearConstraint < Constraint
  % Linear Constraint. lb <= Ax <= ub. Support the linear constraint being sparse
  properties(SetAccess = protected)
    A % A matrix of size num_cnstr x xdim. The linear constraint matrix
    A_val % A double vector. The value of the non-zero entries of the
          % gradient matrix. These correspond to iCfun and jCvar parameters from
          % the superclass
  end


  methods
    function obj = LinearConstraint(lb,ub,A)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param A     -- A matrix of size num_cnstr x xdim. The linear constraint matrix
      xdim = size(A,2);
      obj = obj@Constraint(lb,ub,xdim,2);
      if(~isnumeric(lb) || ~isnumeric(ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint lb and ub should be numeric')
      end
      if(~isnumeric(A))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint A should be numeric');
      end
      sizecheck(A,[obj.num_cnstr,obj.xdim]);
      obj.A = A;
      [iAfun,jAvar,obj.A_val] = find(obj.A);
      obj = obj.setSparseStructure(iAfun,jAvar);
      obj.A = sparse(A);
    end

    function obj = setName(obj,name)
      % @param name   -- A cell array, name{i} is the name string of i'th constraint
      if(~iscell(name))
        error('Drake:LinearConstraint:name should be a cell array');
      end
      sizecheck(name,[obj.num_cnstr,1]);
      if(~all(cellfun(@ischar,name)))
        error('Drake:LinearConstraint:name should be a cell array of strings');
      end
      obj.name = name;
    end
  end

  methods (Access = protected)
    function [c,dc,ddc] = constraintEval(obj,x)
      c = obj.A*x;
      if(nargout>1)
        dc = obj.A;
      end
      if(nargout>2)
        ddc = sparse(size(obj.A,1),obj.xdim^2);
      end
    end
end
end
