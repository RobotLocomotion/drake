classdef NonlinearProgram
  % minimize_x objective(x)
  % subject to
  %            cin_lb<=nonlinear_inequality_constraints(x) <= cin_ub
  %            nonlinear_equality_constraints(x) = 0  
  %            Ain*x <= bin
  %            Aeq*x = beq
  %            x_lb <= x <= x_ub
  properties (SetAccess=protected)
    num_vars % An integer. The number of decision variables
    num_cin % An integer. The number of nonlinear inequality constraints
    num_ceq % An integer. The number of nonlinear equality constraints
    Ain % A double matrix with num_vars columns
    bin % A double vector, with same number of rows as Ain
    Ain_name % A cell of strings. Ain_name{i} is the name of the i'th inequality linear constraint
    Aeq % A double matrix with num_vars columns
    beq % A double vector with the same number of rows as Aeq
    Aeq_name % A cell of strings. Aeq_name{i} is the name of the i'th equality constraint
    cin_lb % A num_cin x 1 double vector. The lower bound of the nonlinear inequality constraint
    cin_ub % A num_cin x 1 double vector. The upper bound of the nonlinear inequality constraint
    cin_name % A cell of num_cin x 1 strings. cin_name{i} is the name of the i'th nonlinear inequality constraint
    ceq_name % A cell of num_ceq x 1 strings. ceq_name{i} is the name of the i'th nonlinear equality constraint
    x_lb % A num_vars x 1 double vector. The lower bound of the decision variables
    x_ub % A num_vars x 1 double vector. The upper bound of the decision variables
    x_name % A cell of num_vars x 1 strings. x_name{i} is the name of the i'th decision variable
    solver % The name of the solver. Currently accept snopt, ipopt and fmincon
    solver_options 
    display_funs
    display_fun_indices
    check_grad % A boolean, True if the user gradient will be checked against
               % numerical gradient at the begining and end of the nonlinear optimization
    constraint_err_tol % A small scaler. Check whether the constraint are satisfied within the tolerance
    
    nlcon % A cell array of NonlinearConstraint
    lcon % A cell array of LinearConstraint
    bbcon % A cell array of BoundingBoxConstraint
    cost % A cell array of NonlinearConstraint or LinearConstraint.
    num_nlcon % number of nonlinear constraints
    num_lcon % number of linear constraints
    
    nlcon_xind % A cell array, nlcon_xind{i} is a cell array of int vectors recording the indices of x that is used in evaluation the i'th NonlinearConstraint
               % nlcon{i}.eval(x(nlcon_xind{i}{1},x(nlcon_xind{i}{2},...)
    nlcon_xind_stacked % a cell array of vectors, the stacked values of nlcon_xind{i}
    cost_xind_cell % A cell array, cost_xind{i} is a cell array of int vectors recording the indices of x that is used in evaluating obj.cost{i}
    cost_xind_stacked % A cell array, cost_xind{i} is an int vector recording the indices of x that is used in evaluating obj.cost{i}
    bbcon_xind % A cell array, bbcon_xind{i} is an int vector recording the indices of x used in i'th BoundingBoxConstraint
    nlcon_ineq_idx % row index of nonlinear inequality constraint
    nlcon_eq_idx % row index of nonlinear equality constraint
    
    % a cell array like nlcon_xind, where shared_data_xind_cell{i} is a
    % cell array of int vectors recording indices used in evaluating the 
    % shared_data_function
    shared_data_xind_cell  
    
    % a cell array of function handles, each of which returns a data object
    % so that shared_data{i} = shared_data_functions(x(shared_data_xind_cell{i}{1}),x(shared_data_xind_cell{i}{2}),...)
    shared_data_functions 
    
    % cell arrays of vectors where nlcon_dataind{i} are indices into the
    % shared_data used by nonlinear constraints and cost functions
    nlcon_dataind 
    cost_dataind
  end
  
  properties (Access=private)
    objcon_logic = false;
  end

  properties (Access = protected)
%     iGfun,jGvar  % sparsity pattern in objective and nonlinear constraints
    iFfun,jFvar  % sparsity pattern in the objective function
    iCinfun,jCinvar  % sparsity pattern in the nonlinear inequality constraints
    iCeqfun,jCeqvar  % sparsity pattern in the nonlinear equality constraints
  end
  
  methods
    function obj = NonlinearProgram(num_vars,x_name)
      % @param num_vars          -- Number of decision variables
      % @param x_name       -- An optional argument. A cell of strings containing the name
      % of each decision variable
      if(nargin<2)
        x_name = cellfun(@(i) sprintf('x%d',i),num2cell((1:obj.num_vars)'),'UniformOutput',false);
      else
        if(~iscellstr(x_name) || numel(x_name) ~= num_vars)
          error('Drake:NonlinearProgra:InvalidArgument','Argument x_name should be a cell containing %d strings',obj.num_vars);
        end
        x_name = x_name(:);
      end
      sizecheck(num_vars,[1,1]);
      obj.num_vars = num_vars;
      obj.num_cin = 0;
      obj.num_ceq = 0;
      obj.x_name = x_name;
      obj.x_lb = -inf(num_vars,1);
      obj.x_ub = inf(num_vars,1);
      obj.Ain = zeros(0,num_vars);
      obj.Aeq = zeros(0,num_vars);
      obj.cin_ub = zeros(obj.num_cin,1);
      obj.cin_lb = -inf(obj.num_cin,1);
      obj.iFfun = ones(obj.num_vars,1);
      obj.jFvar = (1:obj.num_vars)';
      obj.iCinfun = reshape(bsxfun(@times,(1:obj.num_cin)',ones(1,obj.num_vars)),[],1);
      obj.jCinvar = reshape(bsxfun(@times,ones(obj.num_cin,1),(1:obj.num_vars)),[],1);
      obj.iCeqfun = reshape(bsxfun(@times,(1:obj.num_ceq)',ones(1,obj.num_vars)),[],1);
      obj.jCeqvar = reshape(bsxfun(@times,ones(obj.num_ceq,1),(1:obj.num_vars)),[],1);
      
      obj.nlcon = {};
      obj.lcon = {};
      obj.bbcon = {};
      obj.num_nlcon = 0;
      obj.num_lcon = 0;
      obj.nlcon_xind = {};
      obj.nlcon_xind_stacked = {};
      obj.nlcon_ineq_idx = [];
      obj.nlcon_eq_idx = [];
      obj.cost = {};
      obj.cost_xind_cell = {};
      obj.cost_xind_stacked = {};
      obj.cin_name = {};
      obj.ceq_name = {};
      obj.Ain_name = {};
      obj.Aeq_name = {};
      
      obj.shared_data_xind_cell = {};
      obj.shared_data_functions = {};
      obj.nlcon_dataind = {};
      obj.cost_dataind = {};
      
      if checkDependency('snopt')
        obj = obj.setSolver('snopt');
      else % todo: check for fmincon?
        obj = obj.setSolver('fmincon');
      end
      obj.solver_options.fmincon = optimset('Display','off');
      obj.solver_options.snopt = struct();
      obj.solver_options.snopt.MajorIterationsLimit = 1000;
      obj.solver_options.snopt.MinorIterationsLimit = 500;
      obj.solver_options.snopt.IterationsLimit = 10000;
      obj.solver_options.snopt.MajorOptimalityTolerance = 1e-6;
      obj.solver_options.snopt.MajorFeasibilityTolerance = 1e-6;
      obj.solver_options.snopt.MinorFeasibilityTolerance = 1e-6;
      obj.solver_options.snopt.SuperbasicsLimit = 300;
      obj.solver_options.snopt.VerifyLevel = 0;
      obj.solver_options.snopt.DerivativeOption = 1;
      obj.solver_options.snopt.print = '';
      obj.solver_options.snopt.scaleoption = 0;
      obj.solver_options.snopt.NewBasisFile = 0;
      obj.solver_options.snopt.OldBasisFile = 0;
      obj.solver_options.snopt.BackupBasisFile = 0;
      obj.solver_options.snopt.LinesearchTolerance = 0.9;
      obj.constraint_err_tol = 1e-4;
      obj.check_grad = false;
    end
    
    function obj = addCompositeConstraints(obj,cnstr,xind,data_ind)
      % add a CompositeConstraint to the object, change the constraint evalation of the
      % program.
      % @param mgr     -- A CompositeConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      % @param data_ind  -- Optional argument. shared_data{data_ind} are the data objects used
      if(~isa(cnstr,'CompositeConstraint'))
        error('Drake:NonlinearProgram:UnsupportedConstraint','addCompositeConstraints expects a CompositeConstraint object');
      end
      if(nargin<3)
        xind = {(1:obj.num_vars)'};
      end
      if ~iscell(xind)
        xind = {xind(:)};
      end
      if size(xind,1) < size(xind,2)
        xind = xind';
      end
      if size(xind,2) ~= 1
        error('Drake:NonlinearProgram:InvalidArgument','xind must be a 1-D vector or 1-D cell array');
      end

      % add in slack variables to end, and adjust xind accordingly
      n_slack = cnstr.n_slack;
      for i=1:length(xind)
        xind{i} = [xind{i};(obj.num_vars + 1 : obj.num_vars + n_slack)'];
      end
      obj = obj.addDecisionVariable(n_slack);
      
      if nargin < 4
        args = {xind};
      else
        args = {xind,data_ind};
      end
      
      % add constraints
      for k=1:length(cnstr.constraints),
        obj = obj.addConstraint(cnstr.constraints{k}, args{:});
      end      
    end
    
    function obj = addConstraint(obj,cnstr,varargin)
      % obj = addConstraint(obj,cnstr,varargin)
      % Queries the constraint type and calls the appropriate addConstraint
      % method (e.g. addLinearConstraint, etc)
      %
      % @param cnstr a Constraint object.  if cnstr is a cell array, then
      % each of the constraints are added individually.
      % @param varargin the remaining arguments are passed directly through
      % to the specialized methods. Note that if cnstr is a cell array,
      % then the same varargin is passed to all of the specialized methods.
      
      if iscell(cnstr)
        for i=1:numel(cnstr)
          obj = addConstraint(obj,cnstr{i},varargin{:});
        end
      elseif isa(cnstr,'BoundingBoxConstraint')
        obj = addBoundingBoxConstraint(obj,cnstr,varargin{:});
      elseif isa(cnstr,'LinearConstraint')
        obj = addLinearConstraint(obj,cnstr,varargin{:});
      elseif isa(cnstr,'CompositeConstraint')
        obj = addCompositeConstraints(obj,cnstr,varargin{:});
      elseif isa(cnstr,'Constraint')
        obj = addNonlinearConstraint(obj,cnstr,varargin{:});
      else
        error('Drake:NonlinearProgram:UnsupportedConstraint','Unsupported constraint type');
      end
    end
    
    function obj = addNonlinearConstraint(obj,cnstr,xind, data_ind)
      % add a NonlinearConstraint to the object, change the constraint evalation of the
      % program. 
      % @param cnstr     -- A NonlinearConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      % @param data_ind  -- Optional argument. shared_data{data_ind} are the data objects used
      if(nargin<3)
        xind = {(1:obj.num_vars)'};
      end
      if ~iscell(xind)
        xind = {xind(:)};
      end
      if size(xind,1) < size(xind,2)
        xind = xind';
      end
      if size(xind,2) ~= 1
        error('Drake:NonlinearProgram:InvalidArgument','xind must be a 1-D vector or 1-D cell array');
      end
      
      xind_vec = cell2mat(xind);
      
      if(nargin<4)
        data_ind = [];
      end
      data_ind = data_ind(:);
      
      if(~isa(cnstr,'Constraint'))
        error('Drake:NonlinearProgram:UnsupportedConstraint','addNonlinearConstraint expects a Constraint object');
      end
      if length(xind_vec) ~= cnstr.xdim
        error('Drake:NonlinearProgram:InvalidArgument','the length of xind must match the x-dimension of the constraint');
      end
%       obj.nlcon = [obj.nlcon,{cnstr}];
      obj.nlcon{end+1} = cnstr;
      
      obj.cin_ub = [obj.cin_ub;cnstr.ub(cnstr.cin_idx)];
      obj.cin_lb = [obj.cin_lb;cnstr.lb(cnstr.cin_idx)];
      obj.nlcon_ineq_idx = [obj.nlcon_ineq_idx;obj.num_nlcon+cnstr.cin_idx];
      obj.nlcon_eq_idx = [obj.nlcon_eq_idx;obj.num_nlcon+cnstr.ceq_idx];
      Geq_idx = cnstr.lb(cnstr.iCfun) == cnstr.ub(cnstr.iCfun);
      Gin_idx = ~Geq_idx;
      inv_ceq_idx = zeros(cnstr.num_cnstr,1);
      inv_ceq_idx(cnstr.ceq_idx) = (1:length(cnstr.ceq_idx))';
      inv_cin_idx = zeros(cnstr.num_cnstr,1);
      inv_cin_idx(cnstr.cin_idx) = (1:length(cnstr.cin_idx))';
      obj.iCinfun = [obj.iCinfun;obj.num_cin+inv_cin_idx(cnstr.iCfun(Gin_idx))];
      obj.jCinvar = [obj.jCinvar;xind_vec(cnstr.jCvar(Gin_idx))];
      obj.iCeqfun = [obj.iCeqfun;obj.num_ceq+inv_ceq_idx(cnstr.iCfun(Geq_idx))];
      obj.jCeqvar = [obj.jCeqvar;xind_vec(cnstr.jCvar(Geq_idx))];
      obj.cin_name = [obj.cin_name;cnstr.name(cnstr.cin_idx)];
      obj.ceq_name = [obj.ceq_name;cnstr.name(cnstr.ceq_idx)];
      obj.num_cin = obj.num_cin + length(cnstr.cin_idx);
      obj.num_ceq = obj.num_ceq + length(cnstr.ceq_idx);
      obj.num_nlcon = obj.num_nlcon + cnstr.num_cnstr;
      obj.nlcon_xind{end+1} = xind;
      obj.nlcon_xind_stacked{end+1} = xind_vec;
      obj.nlcon_dataind{end+1} = data_ind;
    end
    
    function obj = addLinearConstraint(obj,cnstr,xind)
      % add a LinearConstraint to the program
      % @param cnstr     -- A LinearConstraint object
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the constraint. Default value is (1:obj.num_vars)
      % @param cnstr_name  -- An optional argument. A cell of strings. cnstr_name{i} is
      % the name of the i'th constraint. If not given, the cnstr.name will be used instead
      if cnstr.num_cnstr > 0
        if(nargin<3)
          xind = (1:obj.num_vars)';
        end
        if iscell(xind)
          xind = cell2mat(xind);
        end
        xind = xind(:);
        if(~isa(cnstr,'LinearConstraint'))
          error('Drake:NonlinearProgram:UnsupportedConstraint','addLinearConstraint expects a LinearConstraint object');
        end
        if length(xind) ~= cnstr.xdim
          error('Drake:NonlinearProgram:InvalidArgument','the length of xind must match the x-dimension of the constraint');
        end
        obj.lcon = [obj.lcon,{cnstr}];

        cnstr_A = sparse(cnstr.iCfun,xind(cnstr.jCvar),cnstr.A_val,cnstr.num_cnstr,obj.num_vars,cnstr.nnz);
        cnstr_beq = (cnstr.lb(cnstr.ceq_idx)+cnstr.ub(cnstr.ceq_idx))/2;
        cnstr_Aeq = cnstr_A(cnstr.ceq_idx,:);
        cnstr_Ain = cnstr_A(cnstr.cin_idx,:);
        cnstr_ineq_name = cnstr.name(cnstr.cin_idx);
        cnstr_bin_lb = cnstr.lb(cnstr.cin_idx);
        cnstr_bin_ub = cnstr.ub(cnstr.cin_idx);
        bin_ub_not_inf_idx = ~isinf(cnstr_bin_ub);
        bin_lb_not_inf_idx = ~isinf(cnstr_bin_lb);
        if(sum(bin_ub_not_inf_idx | bin_lb_not_inf_idx)>0)
          obj.Ain = vertcat(obj.Ain,[cnstr_Ain(bin_ub_not_inf_idx,:);-cnstr_Ain(bin_lb_not_inf_idx,:)]);
          obj.bin = vertcat(obj.bin,[cnstr_bin_ub(bin_ub_not_inf_idx);-cnstr_bin_lb(bin_lb_not_inf_idx)]);
          obj.Ain_name = [obj.Ain_name;cnstr_ineq_name(bin_ub_not_inf_idx);cnstr_ineq_name(bin_lb_not_inf_idx)];
        end
        obj.Aeq_name = [obj.Aeq_name;cnstr.name(cnstr.ceq_idx)];
        if(numel(cnstr_Aeq)>0)
          obj.Aeq = vertcat(obj.Aeq,cnstr_Aeq);
          obj.beq = vertcat(obj.beq,cnstr_beq);
        end
      end
    end   

    function obj = addBoundingBoxConstraint(obj,cnstr,xind)
      % add a BoundingBoxConstraint to the program
      % @param cnstr      -- A BoundingBoxConstraint
      % @param xind       -- Optional argument. x(xind) is the decision variables to be
      % set bounds
      if(nargin < 3)
        xind = (1:obj.num_vars)';
      end
      if iscell(xind)
        xind = cell2mat(xind);
      end
      xind = xind(:);
      if(~isa(cnstr,'BoundingBoxConstraint'))
        error('Drake:NonlinearProgram:UnsupportedConstraint','addBoundingBoxConstraint expects a BoundingBoxConstraint object');
      end
      if length(xind) ~= cnstr.xdim
        error('Drake:NonlinearProgram:InvalidArgument','the length of xind must match the x-dimension of the constraint');
      end
      obj.bbcon = [obj.bbcon,{cnstr}];
      obj.x_lb(xind) = max([cnstr.lb obj.x_lb(xind)],[],2);
      obj.x_ub(xind) = min([cnstr.ub obj.x_ub(xind)],[],2);
    end
    
    function obj = addCost(obj,cnstr,xind,data_ind)
      % Add a cost to the objective function
      % @param cnstr   -- A NonlinearConstraint or a LinearConstraint
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      % @param data_ind  -- Optional argument. shared_data{data_ind} are the data objects used
      if(nargin<3)
        xind = {(1:obj.num_vars)'};
      end
      if ~iscell(xind)
        xind = {xind(:)};
      end
      xind_vec = cell2mat(xind);
      if(nargin<4)
        data_ind = [];
      end
      data_ind = data_ind(:);
      if ~isa(cnstr,'Constraint')
        error('Drake:NonlinearProgram:UnsupportedConstraint','addCost expects a Constraint object');
      end
      
      if(isa(cnstr,'LinearConstraint'))
        % Treat linear constraints differently
        if(cnstr.num_cnstr ~= 1)
          error('Drake:NonlinearProgram:WrongCost','addCost only accept scalar function');
        end
        obj.cost = [obj.cost,{cnstr}];
        obj.cost_xind_cell{end+1} = {xind_vec(cnstr.jCvar);};
        obj.cost_xind_stacked{end+1} = xind_vec(cnstr.jCvar);
        obj.cost_dataind{end+1} = data_ind;
        obj.jFvar = unique([obj.jFvar;xind_vec(cnstr.jCvar)]);
        obj.iFfun = ones(length(obj.jFvar),1);
      else
        if(cnstr.num_cnstr ~= 1)
          error('Drake:NonlinearProgram:WrongCost','addCost only accept scalar function');
        end
        obj.cost = [obj.cost,{cnstr}];
        obj.cost_xind_cell{end+1} = xind;
        obj.cost_xind_stacked{end+1} = xind_vec;
        obj.cost_dataind{end+1} = data_ind;
%         obj.cost_xind_cell = [obj.cost_xind_cell,{xind(cnstr.jCvar)}];
        obj.jFvar = unique([obj.jFvar;xind_vec(cnstr.jCvar)]);
        obj.iFfun = ones(length(obj.jFvar),1);
      end
    end
    
    function args = getArgumentArray(obj,x,xind)
      % Retrieves the elements from the vector x related to xind and returns
      % them as a cell array where:
      % args{i} = x(xind{i})
      narg = length(xind);
      args = cell(narg,1);
      for j=1:narg,
        args{j} = x(xind{j});
      end
    end
    
    function [g,h,dg,dh] = nonlinearConstraints(obj,x)
      % evaluate the nonlinear constraints
      % @param x  A num_vars x 1 double vector. The decision variables
      % @retval g  The value of the nonlinear inequality constraints
      % @retval h  The value of the nonlinear equality constraints
      % @retval dg  The gradient of g w.r.t x
      % @retval dh  The gradient of h w.r.t x
      shared_data = obj.evaluateSharedDataFunctions(x);
      f = zeros(obj.num_nlcon,1);
      G = zeros(obj.num_nlcon,obj.num_vars);
      f_count = 0;
      for i = 1:length(obj.nlcon)
        args = [getArgumentArray(obj,x,obj.nlcon_xind{i});shared_data(obj.nlcon_dataind{i})];
        if(nargout>2)
          [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind_stacked{i})] = ...
            obj.nlcon{i}.eval(args{:});
        else
          f(f_count+(1:obj.nlcon{i}.num_cnstr)) = obj.nlcon{i}.eval(args{:});
        end
        f(f_count+obj.nlcon{i}.ceq_idx) = f(f_count+obj.nlcon{i}.ceq_idx)-obj.nlcon{i}.ub(obj.nlcon{i}.ceq_idx);
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      g = f(obj.nlcon_ineq_idx);
      h = f(obj.nlcon_eq_idx);
      if(nargout>2)
        dg = G(obj.nlcon_ineq_idx,:);
        dh = G(obj.nlcon_eq_idx,:);
      end
    end
     
    function [f,df] = objective(obj,x)
      % return the value of the objective
      % @param x  A obj.num_vars x 1 double vector. The decision variables
      % @retval f  A double scalar. The value of the objective function
      % @retval df  The gradient of f w.r.t x
      shared_data = obj.evaluateSharedDataFunctions(x);

      for i=1:length(obj.display_funs)
        obj.display_funs{i}(x(obj.display_fun_indices{i}));
      end
      
      f = 0;
      df = zeros(1,obj.num_vars);
      for i = 1:length(obj.cost)
        args = [getArgumentArray(obj,x,obj.cost_xind_cell{i});shared_data(obj.cost_dataind{i})];
        if(nargout>1)
          [fi,dfi] = obj.cost{i}.eval(args{:});
        else
          fi = obj.cost{i}.eval(args{:});
        end
        f = f+fi;
        if(nargout>1)
          df(obj.cost_xind_stacked{i}) = df(obj.cost_xind_stacked{i})+dfi;
        end
      end
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      % evaluate the objective and the nonlinear constraints altogher
      % @param x   A obj.num_vars x 1 double vector. The decision variables
      % @retval f   A 1+obj.num_cin+obj.num_ceq x 1 double vector. f =
      % [objective;nonlinear_inequality_constraints;nonlinear_equality_constraints]
      % @retval df  The gradient of f w.r.t x
      shared_data = obj.evaluateSharedDataFunctions(x);
      
      for i=1:length(obj.display_funs)
        obj.display_funs{i}(x(obj.display_fun_indices{i}));
      end
      
      f = zeros(1+obj.num_nlcon,1);
      G = zeros(1+obj.num_nlcon,obj.num_vars);
      for i = 1:length(obj.cost)
        args = [getArgumentArray(obj,x,obj.cost_xind_cell{i});shared_data(obj.cost_dataind{i})];
        if(nargout>1)
          [fi,dfi] = obj.cost{i}.eval(args{:});
        else
          fi = obj.cost{i}.eval(args{:});
        end
        f(1) = f(1)+fi;
        if(nargout>1)
          G(1,obj.cost_xind_stacked{i}) = G(1,obj.cost_xind_stacked{i})+dfi;
        end
      end
      f_count = 1;
      for i = 1:length(obj.nlcon)
        args = [getArgumentArray(obj,x,obj.nlcon_xind{i});shared_data(obj.nlcon_dataind{i})];
        if(nargout>1)
        [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind_stacked{i})] = ...
          obj.nlcon{i}.eval(args{:});
        else
          f(f_count+(1:obj.nlcon{i}.num_cnstr)) = obj.nlcon{i}.eval(args{:});
        end
        f(f_count+obj.nlcon{i}.ceq_idx) = f(f_count+obj.nlcon{i}.ceq_idx)-obj.nlcon{i}.ub(obj.nlcon{i}.ceq_idx);
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      f = [f(1);f(1+obj.nlcon_ineq_idx);f(1+obj.nlcon_eq_idx)];
      if(nargout>1)
        G = [G(1,:);G(1+obj.nlcon_ineq_idx,:);G(1+obj.nlcon_eq_idx,:)];
      end
    end
    
    function obj = addDecisionVariable(obj,num_new_vars,var_name)
      % appending new decision variables to the end of the current decision variables
      % @param num_new_vars      -- An integer. The newly added decision variable is an
      % num_new_vars x 1 double vector.
      % @param var_name       -- An optional argument. A cell of strings containing the
      % name of the new decision variables
      if(nargin<3)
        var_name = cellfun(@(i) sprintf('x%d',i),num2cell(obj.num_vars+(1:num_new_vars)'),'UniformOutput',false);
      else
        if(~iscellstr(var_name) || numel(var_name) ~= num_new_vars)
          error('Drake:NonlinearProgram:addDecisionVariable:InvalidArgument','Argument var_name should be a cell of %d strings',num_new_vars);
        end
        var_name = var_name(:);
      end
      obj.num_vars = obj.num_vars+num_new_vars;
      obj.x_name = [obj.x_name;var_name];
      obj.x_lb = [obj.x_lb;-inf(num_new_vars,1)];
      obj.x_ub = [obj.x_ub;inf(num_new_vars,1)];
      if(~isempty(obj.Aeq))
        obj.Aeq = [obj.Aeq zeros(length(obj.beq),num_new_vars)];
      end
      if(~isempty(obj.Ain))
        obj.Ain = [obj.Ain zeros(length(obj.bin),num_new_vars)];
      end
    end
    
    function obj = replaceCost(obj,cost,cost_idx,xind)
      % replace the cost_idx'th cost in the original problem with a new cost
      % @param cost     -- A Constraint object, currently accepts NonlinearConstraint and
      % LinearConstraint
      % @param cost_idx -- The index of the original cost to be replaced
      % @param xind     -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      if(nargin<4)
        xind = {(1:obj.num_vars)'};
      end
      if ~iscell(xind)
        xind = {xind(:)};
      end
      obj.iFfun = [];
      obj.jFvar = [];
      num_cost = length(obj.cost);
      sizecheck(cost_idx,[1,1]);
      if(cost_idx>num_cost || cost_idx<1)
        error('Drake:NonlinearProgram:replaceCost:cost_idx is out of range');
      end
      cost_tmp = obj.cost;
      cost_tmp{cost_idx} = cost;
      cost_xind_tmp = obj.cost_xind_cell;
      cost_xind_tmp{cost_idx} = xind;
      obj.cost = {};
      obj.cost_xind_cell = {};
      obj.cost_xind_stacked = {};
      for i = 1:num_cost
        obj = obj.addCost(cost_tmp{i},cost_xind_tmp{i});
      end
    end
    
    function obj = replaceBoundingBoxConstraint(obj,cnstr,cnstr_idx,xind)
      % replace the cnstr_idx'th BoundingBoxConstraint in obj.bb_cnstr with the new cnstr.
      % @param cnstr    -- A BoundingBoxConstraint object
      % @param cnstr_idx  -- THe index of the replaced BoundingBoxConstraint in the
      % obj.bb_cnstr cell
      % @param xind      Optional argument. x(xind) is the decision variables used in
      % evaluating the constraint. Default value is (1:obj.num_vars) 
      if(nargin < 4)
        xind = (1:obj.num_vars)';
      end
      xind = xind(:);
      obj.x_lb = -inf(obj.num_vars,1);
      obj.x_ub = inf(obj.num_vars,1);
      num_bbcon = length(obj.bbcon);
      sizecheck(cnstr_idx,[1,1]);
      if(cnstr_idx>num_bbcon || cnstr_idx <1)
        error('Drake:NonlinearProgram:replaceBoundingBoxConstraint:cnstr_idx is out of range');
      end
      bbcon_tmp = obj.bbcon;
      bbcon_xind_tmp = obj.bbcon_xind;
      bbcon_tmp{cnstr_idx} = cnstr;
      bbcon_xind_tmp{cnstr_idx} = xind;
      obj.bbcon = {};
      obj.bbcon_xind = {};
      for i = 1:length(obj.bbcon)
        obj = obj.addBoundingBoxConstraint(bbcon_tmp{i},bbcon_xind_tmp{i});
      end
    end
    
    function [obj,ind] = addSharedDataFunction(obj,user_fun,xind)
      % Adds the specified shared data function to be evaluated within each iteration of the program     
      % @param user_fun -- The function to be evaluated, where
      %   shared_data{ind} = user_fun(x(xind));
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      %   in evaluating the cnstr. Default value is (1:obj.num_vars)
      % @return ind -- the shared data index
      if(nargin<3)
        xind = {(1:obj.num_vars)'};
      end
      if ~iscell(xind)
        xind = {xind(:)};
      end
      if isa(user_fun,'FunctionWrapper')
        obj.shared_data_functions{end+1} = user_fun;
      else
        obj.shared_data_functions{end+1} = FunctionWrapper(user_fun);
      end
      obj.shared_data_xind_cell{end+1} = xind;
      ind = obj.getNumSharedDataFunctions();
    end
    
    function n = getNumSharedDataFunctions(obj)
      n = length(obj.shared_data_functions);
    end
    
    function data = evaluateSharedDataFunctions(obj,x)
      % Evaluate all shared data functions and return the data object
      nData = length(obj.shared_data_functions);
      data = cell(nData,1);
      for i=1:nData
        args = getArgumentArray(obj,x,obj.shared_data_xind_cell{i});
%         data{i} = obj.shared_data_functions{i}.eval(args{:});
        data{i} = obj.shared_data_functions{i}.eval(args{:});
      end
    end
    
    function obj = addDisplayFunction(obj,display_fun,indices)
      % add a dispay function that gets called on every iteration of the
      % algorithm
      % @param displayFun a function handle of the form displayFun(x(indices))
      % @param indices optionally specify a subset of the decision
      % variables to be passed to the displayFun @default 1:obj.num_vars
      
      typecheck(display_fun,'function_handle');
      if nargin<3, indices = 1:obj.num_vars; end

      obj.display_funs = vertcat(obj.display_funs,{display_fun});
      obj.display_fun_indices = vertcat(obj.display_fun_indices,{indices});
    end
    
    function obj = setCheckGrad(obj,check_grad)
      sizecheck(check_grad,[1,1]);
      obj.check_grad = logical(check_grad);
    end
    
    function obj = setConstraintErrTol(obj,tol)
      if(~isnumeric(tol) || numel(tol) ~= 1)
        error('Drake:NonlinearProgram:setConstraintErrTol:tol should be scaler');
      end
      if(tol<=0)
        error('Drake:NonlinearProgram:setConstraintErrTol:tol should be positive');
      end
      obj.constraint_err_tol = tol;
    end
    
    function obj = setSolver(obj,solver)
      % @param solver  Can be 'snopt', 'ipopt', 'fmincon' and 'default'.
      typecheck(solver,'char');
      if(strcmp(solver,'snopt'))
        if(~checkDependency('snopt'))
          error('Drake:NonlinearProgram:UnsupportedSolver','SNOPT is not installed');
        end
        obj.solver = solver;
      elseif(strcmp(solver,'fmincon'))
        obj.solver = solver;
      elseif(strcmp(solver,'ipopt'))
        if(~checkDependency('ipopt'))
          error('Drake:NonlinearProgram:UnsupportedSolver','Ipopt is not installed yet');
        end
        obj.solver = solver;
      elseif(strcmp(solver,'default'))
        if(checkDependency('snopt'))
          obj = obj.setSolver('snopt');
        else
          obj = obj.setSolver('fmincon');
        end
        
      end
    end
    
    function obj = setSolverOptions(obj,solver,optionname,optionval)
      % @param solver   - string name of the solver
      % @param optionname    -- string name of the option field
      % @param optionval     -- option value
      if(strcmpi(solver,'snopt'))
        if(strcmpi(optionname(~isspace(optionname)),'majorfeasibilitytolerance'))
          sizecheck(optionval,[1,1]);
          if(optionval<=0)
            error('Drake:NonlinearProgram:setSolverOptions:MajorFeasibilityTolerance should be positive');
          end
          obj.solver_options.snopt.MajorFeasibilityTolerance = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'minorfeasibilitytolerance'))
          sizecheck(optionval,[1,1]);
          if(optionval<=0)
            error('Drake:NonlinearProgram:setSolverOptions:MinorFeasibilityTolerance should be positive');
          end
          obj.solver_options.snopt.MinorFeasibilityTolerance = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'majoroptimalitytolerance'))
          sizecheck(optionval,[1,1]);
          if(optionval<=0)
            error('Drake:NonlinearProgram:setSolverOptions:MajorOptimalityTolerance should be positive');
          end
          obj.solver_options.snopt.MajorOptimalityTolerance = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'majoriterationslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:MajorIterationsLimit should be positive integers');
          end
          obj.solver_options.snopt.MajorIterationsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'minoriterationslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:MinorIterationsLimit should be positive integers');
          end
          obj.solver_options.snopt.MinorIterationsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'iterationslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:IterationsLimit should be positive integers');
          end
          obj.solver_options.snopt.IterationsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'superbasicslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:SuperbasicsLimit should be positive integers');
          end
          obj.solver_options.snopt.SuperbasicsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'derivativeoption'))
          sizecheck(optionval,[1,1]);
          if(optionval ~= 0  && optionval ~= 1)
            error('Drake:NonlinearProgram:setSolverOptions:DerivativeOption can be either 0 or 1');
          end
          obj.solver_options.snopt.DerivativeOption = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'verifylevel'))
          sizecheck(optionval,[1,1]);
          if(optionval ~= 0  && optionval ~= 1 && optionval ~= 2 && optionval ~= 3 && optionval ~= -1)
            error('Drake:NonlinearProgram:setSolverOptions:VerifyLevel can be either 0,1,2,3 or -1');
          end
          obj.solver_options.snopt.VerifyLevel = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'print'))
          if(~ischar(optionval))
            error('Drake:NonlinearProgram:setSolverOptions:print should be the file name string');
          end
          obj.solver_options.snopt.print = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'scaleoption'))
          if(~isnumeric(optionval) || numel(optionval) ~= 1)
            error('Drake:NonlinearProgram:setSolverOptions:scaleoption should be a scaler');
          end
          if(optionval ~= 0 && optionval ~= 1 && optionval ~= 2)
            error('Drake:NonlinearProgram:setSolverOptions:scaleoption should be either 0,1 or 2');
          end
          obj.solver_options.snopt.scaleoption = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'oldbasisfile'))
          if(~isnumeric(optionval) || numel(optionval) ~= 1)
            error('Drake:NonlinearProgram:setSolverOptions:OptionVal', 'OldBasisFile should be a scaler');
          end
          obj.solver_options.snopt.OldBasisFile = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'newbasisfile'))
          if(~isnumeric(optionval) || numel(optionval) ~= 1)
            error('Drake:NonlinearProgram:setSolverOptions:OptionVal', 'NewBasisFile should be a scaler');
          end
          obj.solver_options.snopt.NewBasisFile = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'backupbasisfile'))
          if(~isnumeric(optionval) || numel(optionval) ~= 1)
            error('Drake:NonlinearProgram:setSolverOptions:OptionVal', 'BackupBasisFile should be a scaler');
          end
          obj.solver_options.snopt.BackupBasisFile = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'linesearchtolerance'))
          if(~isnumeric(optionval) || numel(optionval) ~= 1)
            error('Drake:NonlinearProgram:setSolverOptions:scaleoption should be a scaler');
          end
          if(optionval < 0 || optionval > 1)
            error('Drake:NonlinearProgram:setSolverOptions:OptionVal', 'LinesearchTolerance should be between 0 and 1');
          end
          obj.solver_options.snopt.LinesearchTolerance = optionval;
        end
      elseif(strcmpi((solver),'fmincon'))
        obj.solver_options.fmincon = optimset(obj.solver_options.fmincon, optionname, optionval);
      else
        error('solver %s not supported yet',solver);
      end
    end
    
    function [iGfun,jGvar] = getNonlinearGradientSparsity(obj)
      % This function sets the nonlinear sparsity vector iGfun and jGvar based on the
      % nonlinear sparsity of the objective, nonlinear inequality constraints and
      % nonlinear equality constraints
      % @param iGfun,jGvar. G(iGfun,jGvar) are the non-zero entries in the matrix G, which
      % is the gradient of return value f in the objectiveAndNonlinearConstraints function
      iGfun = [obj.iFfun;obj.iCinfun+1;obj.iCeqfun+1+obj.num_cin];
      jGvar = [obj.jFvar;obj.jCinvar;obj.jCeqvar];
    end
    
    function [lb,ub] = bounds(obj)
      % return the bounds for all the objective function, nonlinear constraints and linear
      % constraints
      lb = [-inf;obj.cin_lb;zeros(obj.num_ceq,1);-inf(length(obj.bin),1);obj.beq];
      ub = [inf;obj.cin_ub;zeros(obj.num_ceq,1);obj.bin;obj.beq];
    end
    
    function [x,objval,exitflag,infeasible_constraint_name] = solve(obj,x0)
      % @param x0   A obj.num_vars x 1 double vector. The initial seed
      % @retval x   A obj.num_vars x 1 double vector. The solution obtained after running the
      % solver
      % @retval objval  A double scaler. The value of the objective function after running the
      % solver
      % @retval exitflag   An integer scaler.
      %                    *********************
      %                    If the solver is SNOPT, then exitflag is the same as the INFO returned by
      %                    the solver. Please refer to
      %                    http://www.cam.ucsd.edu/~peg/papers/sndoc7.pdf for more information
      %                    1  -- Successfully solved through SNOPT
      %                    2  -- Solved with SNOPT, but the accuracy of the linear constraints
      %                    cannot be achieved.
      %                    3  -- Solved with SNOPT, but the accuracy of the nonlinear constraints
      %                    cannot be achieved.
      %                    4  -- SNOPT thinks it fails to solve the problem, but the solution
      %                    satisfies the constraints within obj.constraint_err_tol
      %                    5  -- SNOPT thinks it runs out of iterations limits, but the solution
      %                    satisfies the constraints within obj.constraint_err_tol, try increase the
      %                    iterations limits
      %                    6  -- SNOPT thinks it runs out of major iterations limits, but the
      %                    solution satisfies the constraints within obj.constraint_err_tol. try
      %                    increase the major iterations limits
      %                    12 -- SNOPT fails as the linear constraints are infeasible
      %                    13 -- SNOPT fails as the nonlinear constraints are infeasible
      %                    31 -- SNOPT fails by running out of iterations limit
      %                    32 -- SNOPT fails by running out of major iterations limit
      %                    33 -- SNOPT fails due to small super basics limit
      %                    41 -- SNOPT fails due to numerical problems
      %                    *********************
      %                    If the solver is fmincon, then the exitflag is 200 + fmincon_exitflag
      %                    200 -- In fmincon, number of iterations exceeds options.MaxIter
      %                    201 -- In fmincon, the problem is solved successfully
      %                    199 -- In fmincon, stopped by an output function or plot function
      %                    198 -- In fmincon, no feasible point was found.
      %                    202 -- In fmincon, change in x was less than options.TolX and maximum constraint violation was less than options.TolCon.
      %                    203 -- In fmincon, change in the objective function value was less than options.TolFun and maximum constraint violation was less than options.TolCon.
      %                    204 -- In fmincon, magnitude of the search direction was less than 2*options.TolX and maximum constraint violation was less than options.TolCon.
      %                    205 -- In fmincon, magnitude of directional derivative in search
      %                    direction was less than 2*options.TolFun and maximum constraint violation was less than options.TolCon.
      %                    197 -- In fmincon, objective function at current iteration went below options.ObjectiveLimit and maximum constraint violation was less than options.TolCon.
      %                    **********************
      %                    If the solver is IPOPT, then the exitflag = -100 + ipopt_exitflag
      %                    -100  -- solved by Ipopt
      %                    -99   -- In ipopt, solved to acceptable level
      %                    -98   -- In ipopt, infeasible problem detected
      %                    -97   -- In ipopt, search direction becomes too small
      %                    -96   -- In ipopt, diverging iterates
      %                    -95   -- In ipopt, user requested stop
      %                    -101  -- In ipopt, maximum number of iterations exceeded
      %                    -102  -- In ipopt, restoration phase failed
      %                    -103  -- In ipopt, error in step computation
      %                    -110  -- In ipopt, not enough degrees of freedom
      %                    -111  -- In ipopt, invalid problem definition
      %                    -112  -- In ipopt, invalid option
      %                    -113  -- In ipopt, invalid number detected
      %                    -200  -- In ipopt, unrecoverable exception
      %                    -201  -- In ipopt, non-IPOPT exception thrown
      %                    -202  -- In ipopt, insufficient memory
      %                    -299  -- In ipopt, internal error
      switch lower(obj.solver)
        case 'snopt'
          [x,objval,exitflag,infeasible_constraint_name] = snopt(obj,x0);
        case 'fmincon'
          [x,objval,exitflag,infeasible_constraint_name] = fmincon(obj,x0);
        case 'ipopt'
          [x,objval,exitflag,infeasible_constraint_name] = ipopt(obj,x0);
        otherwise
          error('Drake:NonlinearProgram:UnknownSolver',['The requested solver, ',obj.solver,' is not known, or not currently supported']);
      end
    end
    
    function [x,objval,exitflag,execution_time] = compareSolvers(obj,x0,solvers)
      if nargin<3
        solvers = {'fmincon'};
        if(checkDependency('snopt'))
          solvers = [solvers,{'snopt'}];
        end
        if(checkDependency('ipopt'))
          solvers = [solvers,{'ipopt'}];
        end
      end
       
      fprintf('    solver        objval        exitflag   execution time\n-------------------------------------------------------------\n')
      typecheck(solvers,'cell');
      x = cell(1,length(solvers));
      objval = cell(1,length(solvers));
      exitflag = cell(1,length(solvers));
      execution_time = cell(1,length(solvers));
      for i=1:length(solvers)
        obj = obj.setSolver(solvers{i});
        try 
          tic;
          [x{i},objval{i},exitflag{i}] = solve(obj,x0);
          execution_time{i} = toc;
        catch ex
          if ((strncmp(ex.identifier,'Drake:MissingDependency',23)))
            continue;
          else
            rethrow(ex);
          end
        end
        
        fprintf('%12s%12.3f%12d%17.4f\n',solvers{i},objval{i},exitflag{i},execution_time{i});
      end
    end
  end
  
  methods(Access=protected)
    
    function [x,objval,exitflag,infeasible_constraint_name] = snopt(obj,x0)

      global SNOPT_USERFUN;
      SNOPT_USERFUN = @snopt_userfun;
      % Find out the decision variables with lower and upper bounds being equal, remove
      % those decision variables
      x_idx = (1:obj.num_vars)';
      free_x_idx = x_idx(obj.x_lb~=obj.x_ub);
      fix_x_idx = x_idx(obj.x_lb == obj.x_ub);
      x_fix = obj.x_lb(fix_x_idx);
      num_x_free = length(free_x_idx);
      x2freeXmap = zeros(obj.num_vars,1);
      x2freeXmap(free_x_idx) = (1:num_x_free)';
      x_lb_free = obj.x_lb(free_x_idx);
      x_ub_free = obj.x_ub(free_x_idx);
      x0_free = x0(free_x_idx);
      
      if(~isempty(obj.Ain))
        Ain_free = obj.Ain(:,free_x_idx);
        if any(fix_x_idx)
          bin_free = obj.bin-obj.Ain(:,fix_x_idx)*x_fix;
        else
          bin_free = obj.bin;
        end
      else
        Ain_free = [];
        bin_free = [];
      end
      if(~isempty(obj.Aeq))
        Aeq_free = obj.Aeq(:,free_x_idx);
        if any(fix_x_idx)
          beq_free = obj.beq-obj.Aeq(:,fix_x_idx)*x_fix;
        else
          beq_free = obj.beq;
        end
      else
        Aeq_free = [];
        beq_free = [];
      end
      
      [iGfun,jGvar] = obj.getNonlinearGradientSparsity();
      if any(fix_x_idx)
        jGvar_free_idx = all(bsxfun(@minus,jGvar,reshape(fix_x_idx,1,[])),2);
      else
        jGvar_free_idx = true(size(jGvar));
      end
      iGfun_free = iGfun(jGvar_free_idx);
      jGvar_free = x2freeXmap(jGvar(jGvar_free_idx));
      
      A_free = [Ain_free;Aeq_free];
      snseti('Major Iterations Limit',obj.solver_options.snopt.MajorIterationsLimit);
      snseti('Minor Iterations Limit',obj.solver_options.snopt.MinorIterationsLimit);
      snsetr('Major Optimality Tolerance',obj.solver_options.snopt.MajorOptimalityTolerance);
      snsetr('Major Feasibility Tolerance',obj.solver_options.snopt.MajorFeasibilityTolerance);
      snsetr('Minor Feasibility Tolerance',obj.solver_options.snopt.MinorFeasibilityTolerance);
      snseti('Superbasics Limit',obj.solver_options.snopt.SuperbasicsLimit);
      snseti('Derivative Option',obj.solver_options.snopt.DerivativeOption);
      snseti('Verify level',obj.solver_options.snopt.VerifyLevel);
      snseti('Iterations Limit',obj.solver_options.snopt.IterationsLimit);
      snseti('Scale option',obj.solver_options.snopt.scaleoption);
      snseti('New Basis File',obj.solver_options.snopt.NewBasisFile);
      snseti('Old Basis File',obj.solver_options.snopt.OldBasisFile);
      snseti('Backup Basis File',obj.solver_options.snopt.BackupBasisFile);
      snsetr('Linesearch tolerance',obj.solver_options.snopt.LinesearchTolerance);

      function [f,G] = snopt_userfun(x_free)
        x_all = zeros(obj.num_vars,1);
        x_all(free_x_idx) = x_free;
        x_all(fix_x_idx) = x_fix;
        [f,G] = geval(@obj.objectiveAndNonlinearConstraints,x_all);
        f = [f;zeros(length(bin_free)+length(beq_free),1)];
        
        G = G(sub2ind(size(G),iGfun,jGvar));
        G = G(jGvar_free_idx);
        G = G(:);
      end      

      function checkGradient(x_free)
        num_rows_G = 1+obj.num_ceq+obj.num_cin+length(obj.beq)+length(obj.bin);
        [f,G] = snopt_userfun(x_free);
        [~,G_numerical] = geval(@snopt_userfun,x_free,struct('grad_method','numerical'));
        G_user = full(sparse(iGfun_free,jGvar_free,G,num_rows_G,num_x_free));
        G_err = G_user-G_numerical;
        [max_err,max_err_idx] = max(abs(G_err(:)));
        [max_err_row,max_err_col] = ind2sub([num_rows_G,obj.num_vars],max_err_idx);
        display(sprintf('maximum gradient error is in row %d for x[%d], with error %f\nuser gradient %f, numerical gradient %f',...
          max_err_row,max_err_col,max_err,G_user(max_err_row,max_err_col),G_numerical(max_err_row,max_err_col)));
      end
      
      if isempty(A_free)
        Avals = [];
        iAfun = [];
        jAvar = [];
      else
        [iAfun,jAvar,Avals] = find(A_free);
        iAfun = iAfun(:);
        jAvar = jAvar(:);
        Avals = Avals(:);
        iAfun = iAfun + 1 + obj.num_cin + obj.num_ceq;
      end
        
      
      lb = [-inf;obj.cin_lb;zeros(obj.num_ceq,1);-inf(length(bin_free),1);beq_free];
      ub = [inf;obj.cin_ub;zeros(obj.num_ceq,1);bin_free;beq_free];
      if(obj.check_grad)
        display('check the gradient for the initial guess');
        checkGradient(x0_free);
      end
      
      if(~isempty(obj.solver_options.snopt.print))
        snprint(obj.solver_options.snopt.print);
      end
      [x_free,objval,exitflag] = snopt(x0_free, ...
        x_lb_free,x_ub_free, ...
        lb,ub,...
        'snoptUserfun',...
        0,1,...
        Avals,iAfun,jAvar,...
        iGfun_free,jGvar_free);
      if(obj.check_grad)
        display('check the gradient for the SNOPT solution');
        checkGradient(x_free);
      end
      x = zeros(obj.num_vars,1);
      x(free_x_idx) = x_free;
      x(fix_x_idx) = x_fix;
      objval = objval(1);
      [exitflag,infeasible_constraint_name] = obj.mapSolverInfo(exitflag,x);
    end
    
    function [x,objval,exitflag,infeasible_constraint_name] = fmincon(obj,x0)
%       if (obj.num_cin + obj.num_ceq)
%         nonlinearConstraints = @obj.nonlinearConstraint;
%       else
%         nonlinearConstraints = [];
%       end
      ub_inf_idx = isinf(obj.cin_ub);
      lb_inf_idx = isinf(obj.cin_lb);
      function [c,ceq,dc,dceq] = fmincon_userfun(x)
        [g,h,dg,dh] = obj.nonlinearConstraints(x);
        ceq = h;
        c = [g(~ub_inf_idx)-obj.cin_ub(~ub_inf_idx);obj.cin_lb(~lb_inf_idx)-g(~lb_inf_idx)];
        dc = [dg(~ub_inf_idx,:);-dg(~lb_inf_idx,:)];
        dceq = dh;
      end
      
      [x,objval,exitflag] = fmincon(@obj.objective,x0,obj.Ain,...
        obj.bin,obj.Aeq,obj.beq,obj.x_lb,obj.x_ub,@fmincon_userfun,obj.solver_options.fmincon);
      objval = full(objval);
      
      
      [exitflag,infeasible_constraint_name] = obj.mapSolverInfo(exitflag,x);
    end
    
    function [x,objval,exitflag,infeasible_constraint_name] = ipopt(obj,x0)
      checkDependency('ipopt');
      
      iJfun = [obj.iCinfun;obj.iCeqfun+obj.num_cin];
      jJvar = [obj.jCinvar;obj.jCeqvar];
      A = [obj.Ain;obj.Aeq];
      [iAfun,jAvar] = find(A);
      iJfun = [iJfun;iAfun+obj.num_cin+obj.num_ceq];
      jJvar = [jJvar;jAvar];
      num_constraints = obj.num_cin+obj.num_ceq+length(obj.bin)+length(obj.beq);
      
      function f = objective(x)
        f = obj.objective(x);
      end
      function df = gradient(x)
        [~,df] = obj.objective(x);
      end
      function c = constraints(x)
        [g,h] = obj.nonlinearConstraints(x);
        c = [g;h;obj.Ain*x;obj.Aeq*x];
      end
      function J = jacobian(x)
        [~,~,dg,dh] = obj.nonlinearConstraints(x);
        J = [dg;dh;obj.Ain;obj.Aeq];
        Jval = J(sub2ind([num_constraints,obj.num_vars],iJfun,jJvar));
        J = sparse(iJfun,jJvar,Jval,num_constraints,obj.num_vars);
      end
      
      function J = jacobianstructure()
        J = sparse(iJfun,jJvar,ones(length(iJfun),1),num_constraints,obj.num_vars);
      end
      funcs.objective = @objective;
      funcs.gradient = @gradient;
      funcs.constraints = @constraints;
      funcs.jacobian = @jacobian;
      funcs.jacobianstructure = @jacobianstructure;
      
      options.lb = obj.x_lb;
      options.ub = obj.x_ub;
      options.cl = [obj.cin_lb;zeros(obj.num_ceq,1);-inf(length(obj.bin),1);obj.beq];
      options.cu = [obj.cin_ub;zeros(obj.num_ceq,1);obj.bin;obj.beq];
      options.ipopt.hessian_approximation = 'limited-memory';
      options.ipopt.print_level = 0;
      
      
      [x,info] = ipopt(x0,funcs,options);
      exitflag = info.status;
      [exitflag,infeasible_constraint_name] = obj.mapSolverInfo(exitflag,x);
      objval = objective(x);
    end
    
    function obj = setVarBounds(obj,lb,ub)
      error('Drake:NonlinearProgram:setVarBounds is deprecated, use addConstraint instead');
    end
  end
  
  methods(Access = protected)
    function [exitflag,infeasible_constraint_name] = mapSolverInfo(obj,exitflag,x)
      % Based on the solver information and solution, re-map the info
      % @param exitflag  The info returned from the solver
      % @param x     The solution
      infeasible_constraint_name = {};
      switch obj.solver
        case 'snopt'
          if exitflag~=1, warning('Drake:NonlinearProgram:SNOPTExitFlag',' %3d %s\n',exitflag,snoptInfo(exitflag)); end
          if(exitflag>10)
            infeasible_constraint_name = infeasibleConstraint(obj,x);
            if(isempty(infeasible_constraint_name))              
              if(exitflag == 13 || exitflag == 12)
                exitflag = 4;
              elseif(exitflag == 31)
                exitflag = 5;
              elseif(exitflag == 32)
                exitflag = 6;
              end
            end
          end
          
        case 'fmincon'
          if(exitflag ~= 1)
            infeasible_constraint_name = infeasibleConstraint(obj,x);
            switch (exitflag)
              case 0
                msg='Number of iterations exceeded';
              case -1
                msg='Stopped by an output function or plot function';
              case -2
                msg='No feasible point was found';
              case 2
                msg='Change in x was less than options.TolX and maximum constraint violation was less than options.TolCon';
              case 3
                msg='Change in the objective function value was less than options.TolFun and maximum constraint violation was less than options.TolCon';
              case 4
                msg='Magnitude of the search direction was less than 2*options.TolX and maximum constraint violation was less than options.TolCon';
              case 5
                msg='Magnitude of directional derivative in search direction was less than 2*options.TolFun and maximum constraint violation was less than options.TolCon';
              case -3
                msg='Objective function at current iteration went below options.ObjectiveLimit and maximum constraint violation was less than options.TolCon';
            end        
            warning('Drake:NonlinearProgram:FMINCONExitFlag',' FMINCON %2d %s',exitflag,msg); 
          end
          exitflag = exitflag+200;
          
        case 'ipopt'
          if(exitflag ~= 0 && exitflag ~= 1)
            infeasible_constraint_name = infeasibleConstraint(obj,x);
            switch (exitflag)
              case 2
                msg = 'infeasible problem detected';
              case 3
                msg = 'search direction becomes too small';
              case 4
                msg = 'diverging iterates';
              case 5
                msg = 'user requested stop';
              case -1
                msg = 'maximum number of iterations exceeded';
              case -2
                msg = 'restoration phase failed';
              case -3
                msg = 'error in step computation';
              case -10
                msg = 'not enough degrees of freedom';
              case -11
                msg = 'invalid problem definition';
              case -12
                msg = 'invalid option';
              case -13
                msg = 'invalid number detected';
              case -100
                msg = 'unrecoverable exception';
              case -101
                msg = 'non-IPOPT exception thrown';
              case -102
                msg = 'insufficient memory';
              case -199
                msg = 'internal error';
            end
            warning('Drake:NonlinearProgram:IPOPTExitFlag','Ipopt %4d %s',exitflag,msg);
          end
          exitflag = exitflag-100;
      end
    end 
  end
  
  methods(Access = private)
    function infeasible_constraint_name = infeasibleConstraint(obj,x)
      [g,h] = obj.nonlinearConstraints(x);
      fval = [g;h];
      A = [obj.Ain;obj.Aeq];
      if(~isempty(A))
        fval = [fval;A*x];
      end
      if(~isempty(fval))
        [lb,ub] = obj.bounds();
        lb = lb(2:end);
        ub = ub(2:end);
        ub_err = fval-ub;
        max_ub_err = max(ub_err);
        max_ub_err = max_ub_err*(max_ub_err>0);
        lb_err = lb-fval;
        max_lb_err = max(lb_err);
        max_lb_err = max_lb_err*(max_lb_err>0);
        if(max_ub_err+max_lb_err>2*obj.constraint_err_tol)
          infeasible_constraint_idx = (ub_err>obj.constraint_err_tol) | (lb_err>obj.constraint_err_tol);
          cnstr_name = [obj.cin_name;obj.ceq_name;obj.Ain_name;obj.Aeq_name];
          infeasible_constraint_name = cnstr_name(infeasible_constraint_idx);
        else
          infeasible_constraint_name = {};
        end
      else
        infeasible_constraint_name = {};
      end

    end
  end
end
