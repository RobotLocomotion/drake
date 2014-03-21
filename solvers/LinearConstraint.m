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
  % @param ceq_idx        -- The row index of the equality constraint
  % @param cin_idx        -- The row index of the inequality constraint
  properties(SetAccess = protected)
    num_cnstr
    xdim
    A
    iAfun
    jAvar
    A_val
    nnz
    ceq_idx
    cin_idx
  end
  
  
  methods
    function obj = LinearConstraint(lb,ub,A)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param A     -- A matrix of size num_cnstr x xdim. The linear constraint matrix
      obj = obj@Constraint(lb,ub);
      if(~isnumeric(lb) || ~isnumeric(ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint lb and ub should be numeric')
      end
      obj.lb = obj.lb(:);
      obj.ub = obj.ub(:);
      obj.num_cnstr = numel(obj.lb);
      if(obj.num_cnstr ~= numel(obj.ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint lb and ub should have same number of elements');
      end
      if(any(obj.lb>obj.ub))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint lb should be no larger than ub');
      end
      c_idx = (1:obj.num_cnstr)';
      obj.ceq_idx = c_idx(obj.lb == obj.ub);
      obj.cin_idx = c_idx(obj.lb ~= obj.ub);
      if(~isnumeric(A))
        error('Drake:LinearConstraint:BadInputs','LinearConstraint A should be numeric');
      end
      obj.xdim = size(A,2);
      sizecheck(A,[obj.num_cnstr,obj.xdim]);
      obj.A = A;
      [obj.iAfun,obj.jAvar,obj.A_val] = find(obj.A);
      obj.nnz = length(obj.A_val); 
      obj.A = sparse(obj.iAfun,obj.jAvar,obj.A_val,obj.num_cnstr,obj.xdim,obj.nnz);
      obj.name = repmat({''},obj.num_cnstr,1);
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
end