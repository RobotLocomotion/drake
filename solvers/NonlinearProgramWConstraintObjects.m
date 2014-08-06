classdef NonlinearProgramWConstraintObjects < NonlinearProgram
  % The constraint of this nonlinear program is specified using 'Constraint' class in
  % drake
  %
  % Generally speaking, this class manages the association between
  % Constraint objects and NonlinearPrograms. With the goal
  % of improved performance by avoiding redundant calculations, the
  % Constraint eval function is assumed to be of the form
  % eval(args{:},data{:}) where args are the standard, numerical valued
  % arguments that are a subset of the NLP decision variables. data is a
  % reference to shared data objects that are pre-computed once per
  % iteration and can be shared between Constraint objects. See
  % addSharedDataFunction() for more details
  
  properties(SetAccess = protected)
    cin_name % A cell array of strings. cin_name{i} is the name of i'th nonlinear inequality constraint
    ceq_name % A cell array of strings. ceq_name{i} is the name of i'th nonlinear equality constraint
    Ain_name % A cell array of strings. Ain_name{i} is the name of i'th linear inequality constraint
    Aeq_name % A cell array of strings. Aeq_name{i} is the name of i'th linear equality constraint
    x_name   % A cell array of strings. x_name{i} is the name of i'th decision variable
  end
  
  properties(SetAccess = protected)
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
  
  methods
    function obj = NonlinearProgramWConstraintObjects(num_vars,x_name)
      % @param num_vars     -- The number of decision variables
      % @param x_name       -- An optional argument. A cell of strings containing the name
      % of each decision variable
      obj = obj@NonlinearProgram(num_vars,0,0);
      if(nargin<2)
        x_name = cellfun(@(i) sprintf('x%d',i),num2cell((1:obj.num_vars)'),'UniformOutput',false);
      else
        if(~iscellstr(x_name) || numel(x_name) ~= obj.num_vars)
          error('Drake:NonlinearProgramWConstraintObjects:InvalidArgument','Argument x_name should be a cell containing %d strings',obj.num_vars);
        end
        x_name = x_name(:);
      end
      obj.x_name = x_name;
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
    end
    
    function obj = addCompositeConstraints(obj,cnstr,xind,data_ind)
      % add a CompositeConstraint to the object, change the constraint evalation of the
      % program.
      % @param mgr     -- A CompositeConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      % @param data_ind  -- Optional argument. shared_data{data_ind} are the data objects used
      if(~isa(cnstr,'CompositeConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addCompositeConstraints expects a CompositeConstraint object');
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
        error('Drake:NonlinearProgramWConstraint:InvalidArgument','xind must be a 1-D vector or 1-D cell array');
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
        error('Drake:NonlinearProgramWConstraintObjects:UnsupportedConstraint','Unsupported constraint type');
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
        error('Drake:NonlinearProgramWConstraintObjects:InvalidArgument','xind must be a 1-D vector or 1-D cell array');
      end
      
      xind_vec = cell2mat(xind);
      
      if(nargin<4)
        data_ind = [];
      end
      data_ind = data_ind(:);
      
      if(~isa(cnstr,'Constraint'))
        error('Drake:NonlinearProgramWConstraintObjects:UnsupportedConstraint','addNonlinearConstraint expects a Constraint object');
      end
      if length(xind_vec) ~= cnstr.xdim
        error('Drake:NonlinearProgramWConstraintObjects:InvalidArgument','the length of xind must match the x-dimension of the constraint');
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
        if(nargin<4)
          cnstr_name = cnstr.name;
        else
          if(~iscellstr(cnstr_name))
            error('Drake:NonlinearProgramWConstraintObjects:cnstr_name should be a cell of strings');
          end
        end
        if(nargin<3)
          xind = (1:obj.num_vars)';
        end
        if iscell(xind)
          xind = cell2mat(xind);
        end
        xind = xind(:);
        if(~isa(cnstr,'LinearConstraint'))
          error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addLinearConstraint expects a LinearConstraint object');
        end
        if length(xind) ~= cnstr.xdim
          error('Drake:NonlinearProgramWConstraint:InvalidArgument','the length of xind must match the x-dimension of the constraint');
        end
        obj.lcon = [obj.lcon,{cnstr}];

        cnstr_A = sparse(cnstr.iCfun,xind(cnstr.jCvar),cnstr.A_val,cnstr.num_cnstr,obj.num_vars,cnstr.nnz);
        cnstr_beq = (cnstr.lb(cnstr.ceq_idx)+cnstr.ub(cnstr.ceq_idx))/2;
        cnstr_Aeq = cnstr_A(cnstr.ceq_idx,:);
        cnstr_Ain = cnstr_A(cnstr.cin_idx,:);
        cnstr_bin_lb = cnstr.lb(cnstr.cin_idx);
        cnstr_bin_ub = cnstr.ub(cnstr.cin_idx);
        bin_ub_inf_idx = ~isinf(cnstr_bin_ub);
        bin_lb_inf_idx = ~isinf(cnstr_bin_lb);
        if(sum(bin_ub_inf_idx | bin_lb_inf_idx)>0)
          obj = obj.addLinearInequalityConstraints([cnstr_Ain(bin_ub_inf_idx,:);-cnstr_Ain(bin_lb_inf_idx,:)],...
            [cnstr_bin_ub(bin_ub_inf_idx);-cnstr_bin_lb(bin_lb_inf_idx)]);
        end
        obj.Ain_name = [obj.Ain_name;cnstr_name(cnstr.cin_idx)];
        obj.Aeq_name = [obj.Aeq_name;cnstr_name(cnstr.ceq_idx)];
        obj = obj.addLinearEqualityConstraints(cnstr_Aeq,cnstr_beq);
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
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addBoundingBoxConstraint expects a BoundingBoxConstraint object');
      end
      if length(xind) ~= cnstr.xdim
        error('Drake:NonlinearProgramWConstraint:InvalidArgument','the length of xind must match the x-dimension of the constraint');
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
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addCost expects a Constraint object');
      end
      
      if(isa(cnstr,'LinearConstraint'))
        % Treat linear constraints differently
        if(cnstr.num_cnstr ~= 1)
          error('Drake:NonlinearProgramWConstraint:WrongCost','addCost only accept scalar function');
        end
        obj.cost = [obj.cost,{cnstr}];
        obj.cost_xind_cell{end+1} = {xind_vec(cnstr.jCvar);};
        obj.cost_xind_stacked{end+1} = xind_vec(cnstr.jCvar);
        obj.cost_dataind{end+1} = data_ind;
        obj.jFvar = unique([obj.jFvar;xind_vec(cnstr.jCvar)]);
        obj.iFfun = ones(length(obj.jFvar),1);
      else
        if(cnstr.num_cnstr ~= 1)
          error('Drake:NonlinearProgramWConstraint:WrongCost','addCost only accept scalar function');
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
      shared_data = obj.evaluateSharedDataFunctions(x);
      f = zeros(obj.num_nlcon,1);
      G = zeros(obj.num_nlcon,obj.num_vars);
      f_count = 0;
      for i = 1:length(obj.nlcon)
        args = [getArgumentArray(obj,x,obj.nlcon_xind{i});shared_data(obj.nlcon_dataind{i})];
        [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind_stacked{i})] = ...
          obj.nlcon{i}.eval(args{:});
        f(f_count+obj.nlcon{i}.ceq_idx) = f(f_count+obj.nlcon{i}.ceq_idx)-obj.nlcon{i}.ub(obj.nlcon{i}.ceq_idx);
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      g = f(obj.nlcon_ineq_idx);
      h = f(obj.nlcon_eq_idx);
      dg = G(obj.nlcon_ineq_idx,:);
      dh = G(obj.nlcon_eq_idx,:);
    end
     
    function [f,df] = objective(obj,x)
      shared_data = obj.evaluateSharedDataFunctions(x);

      for i=1:length(obj.display_funs)
        obj.display_funs{i}(x(obj.display_fun_indices{i}));
      end
      
      f = 0;
      df = zeros(1,obj.num_vars);
      for i = 1:length(obj.cost)
        args = [getArgumentArray(obj,x,obj.cost_xind_cell{i});shared_data(obj.cost_dataind{i})];
        [fi,dfi] = obj.cost{i}.eval(args{:});
        f = f+fi;
        df(obj.cost_xind_stacked{i}) = df(obj.cost_xind_stacked{i})+dfi;
      end
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      shared_data = obj.evaluateSharedDataFunctions(x);
      
      for i=1:length(obj.display_funs)
        obj.display_funs{i}(x(obj.display_fun_indices{i}));
      end
      
      f = zeros(1+obj.num_nlcon,1);
      G = zeros(1+obj.num_nlcon,obj.num_vars);
      for i = 1:length(obj.cost)
        args = [getArgumentArray(obj,x,obj.cost_xind_cell{i});shared_data(obj.cost_dataind{i})];
        [fi,dfi] = obj.cost{i}.eval(args{:});
        f(1) = f(1)+fi;
        G(1,obj.cost_xind_stacked{i}) = G(1,obj.cost_xind_stacked{i})+dfi;
      end
      f_count = 1;
      for i = 1:length(obj.nlcon)
        args = [getArgumentArray(obj,x,obj.nlcon_xind{i});shared_data(obj.nlcon_dataind{i})];
        [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind_stacked{i})] = ...
          obj.nlcon{i}.eval(args{:});
        f(f_count+obj.nlcon{i}.ceq_idx) = f(f_count+obj.nlcon{i}.ceq_idx)-obj.nlcon{i}.ub(obj.nlcon{i}.ceq_idx);
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      f = [f(1);f(1+obj.nlcon_ineq_idx);f(1+obj.nlcon_eq_idx)];
      G = [G(1,:);G(1+obj.nlcon_ineq_idx,:);G(1+obj.nlcon_eq_idx,:)];
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
          error('Drake:NonlinearProgramWConstraintObjects:addDecisionVariable:InvalidArgument','Argument var_name should be a cell of %d strings',num_new_vars);
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
        error('Drake:NonlinearProgramWConstraint:replaceCost:cost_idx is out of range');
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
        error('Drake:NonlinearProgramWConstraint:replaceBoundingBoxConstraint:cnstr_idx is out of range');
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
      obj.shared_data_functions{end+1} = FunctionWrapper(user_fun);
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
    
    function obj = setVarBounds(obj,x_lb,x_ub)
      error('Call addBoundingBoxConstraint instead');
    end
    
    function obj = setObjectiveGradientSparsity(obj,jGvar)
      error('Call addCost instead, it encodes the sparsity');
    end
    
    function obj = setNonlinearInequalityConstraintsGradientSparsity(obj,iGfun,jGvar)
      error('addNonlinearConstraint encodes the sparsity already');
    end
    
    function obj = setNonlinearEqualityConstraintsGradientSparsity(obj,iGfun,jGvar)
      error('addNonlinearConstraint encodes the sparsity already');
    end
  end
end
