classdef LinearConstraint < Constraint
  % Linear Constraint. lb <= Ax <= ub. Support the linear constraint being sparse
  % @param num_cnstr     -- An int scalar. The number of constraints
  % @param xdim          -- An int scalar. The decision variable is an xdim x 1 double
  % vector
  % @param iAfun          -- An int vector. The row index of non-zero entries of the
  % gradient matrix
  % @param jAvar          -- An int vector. The column index of the non-zero entries of
  % the gradient matrix
  % @param A_val          -- A double vector. The value of the non-zero entries of the
  % gradient matrix.
  % @param nnz            -- An int scalar. The maximal number of non-zero entries in the
  % gradient matrix
  % @param A              -- A matrix of size num_cnstr x xdim. The linear constraint
  % matrix
  properties(SetAccess = protected)
    num_cnstr
    xdim
    A
    iAfun
    jAvar
    A_val
    nnz
  end
  
  methods
    function obj = LinearConstraint(c_lb,c_ub,x_lb,x_ub,A)
      % @param c_lb    -- The lower bound of the constraint
      % @param c_ub    -- The upper bound of the constraint
      % @param x_lb    -- The lower bound of the decision variables
      % @param x_ub    -- The upper bound of the decision variables
      % @param A       -- A matrix of size num_cnstr x xdim. The linear constraint matrix
      obj = obj@Constraint(c_lb,c_ub,x_lb,x_ub);
      if(~isnumeric(c_lb) || ~isnumeric(c_ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint c_lb and c_ub should be numeric')
      end
      obj.c_lb = obj.c_lb(:);
      obj.c_ub = obj.c_ub(:);
      obj.num_cnstr = numel(obj.c_lb);
      if(obj.num_cnstr ~= numel(obj.c_ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint c_lb and c_ub should have same number of elements');
      end
      if(any(obj.c_lb>obj.c_ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint c_lb should be no larger than c_ub');
      end
      if(~isnumeric(obj.x_lb) || ~isnumeric(obj.x_ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint x_lb and x_ub should be numeric');
      end
      obj.xdim = numel(obj.x_lb);
      obj.x_lb = obj.x_lb(:);
      obj.x_ub = obj.x_ub(:);
      if(obj.xdim ~= numel(obj.x_ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint x_lb and x_ub should have the same number of elements');
      end
      if(any(obj.x_lb>obj.x_ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint x_lb shoulde be no larger than x_ub');
      end
      if(~isnumeric(A))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint A should be numeric');
      end
      sizecheck(A,[obj.num_cnstr,obj.xdim]);
      obj.A = A;
      [obj.iAfun,obj.jAvar,obj.A_val] = find(obj.A);
      obj.nnz = length(obj.A_val); 
      obj.A = sparse(obj.iAfun,obj.jAvar,obj.A_val,obj.num_cnstr,obj.xdim,obj.nnz);
    end
    
    function varargout = eval(obj,x)
      c = obj.A*x;
      varargout{1} = c;
      if(nargout>1)
        dc = obj.A;
        varargout{2} = dc;
      end
      if(nargout>2)
        ddc = sparse(numel(obj.A),obj.xdim);
        varargout{3} = ddc;
      end
    end
    
  end
end