classdef NonlinearProgramWConstraintObjects < NonlinearProgram
  % The constraint of this nonlinear program is specified using 'Constraint' class in
  % drake
  
  properties(SetAccess = protected)
    cin_name % A cell array of strings. cin_name{i} is the name of i'th nonlinear inequality constraint
    ceq_name % A cell array of strings. ceq_name{i} is the name of i'th nonlinear equality constraint
    Ain_name % A cell array of strings. Ain_name{i} is the name of i'th linear inequality constraint
    Aeq_name % A cell array of strings. Aeq_name{i} is the name of i'th linear equality constraint
  end
  
  properties(Access = protected)
    nlcon % A cell array of NonlinearConstraint
    lcon % A cell array of LinearConstraint
    bbcon % A cell array of BoundingBoxConstraint
    cost % A cell array of NonlinearConstraint or LinearConstraint.
    num_nlcon % number of nonlinear constraints
    num_lcon % number of linear constraints
    
    nlcon_xind % A cell array, nlcon_xind{i} is an int vector recording the indices of x that is used in evaluation the i'th NonlinearConstraint
    cost_xind_cell % A cell array, cost_xind{i} is an int vector recording the indices of x that is used in evaluating obj.cost{i}
    bbcon_xind % A cell array, bbcon_xind{i} is an int vector recording the indices of x used in i'th BoundingBoxConstraint
    nlcon_ineq_idx % row index of nonlinear inequality constraint
    nlcon_eq_idx % row index of nonlinear equality constraint
  end
  
  methods
    function obj = NonlinearProgramWConstraintObjects(num_vars)
      % @param num_vars     -- The number of decision variables
      obj = obj@NonlinearProgram(num_vars,0,0);
      obj.nlcon = {};
      obj.lcon = {};
      obj.bbcon = {};
      obj.num_nlcon = 0;
      obj.num_lcon = 0;
      obj.nlcon_xind = {};
      obj.nlcon_ineq_idx = [];
      obj.nlcon_eq_idx = [];
      obj.cost = {};
      obj.cost_xind_cell = {};
      obj.cin_name = {};
      obj.ceq_name = {};
      obj.Ain_name = {};
      obj.Aeq_name = {};
    end
    
    function obj = addNonlinearConstraint(obj,cnstr,xind)
      % add a NonlinearConstraint to the object, change the constraint evalation of the
      % program. 
      % @param cnstr     -- A NonlinearConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_vars)';
      end
      xind = xind(:);
      if(~isa(cnstr,'NonlinearConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addNonlinearConstraint expects a NonlinearConstraint object');
      end
      obj.nlcon = [obj.nlcon,{cnstr}];
      
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
      obj.jCinvar = [obj.jCinvar;xind(cnstr.jCvar(Gin_idx))];
      obj.iCeqfun = [obj.iCeqfun;obj.num_ceq+inv_ceq_idx(cnstr.iCfun(Geq_idx))];
      obj.jCeqvar = [obj.jCeqvar;xind(cnstr.jCvar(Geq_idx))];
      obj.cin_name = [obj.cin_name;cnstr.name(cnstr.cin_idx)];
      obj.ceq_name = [obj.ceq_name;cnstr.name(cnstr.ceq_idx)];
      obj.num_cin = obj.num_cin + length(cnstr.cin_idx);
      obj.num_ceq = obj.num_ceq + length(cnstr.ceq_idx);
      obj.num_nlcon = obj.num_nlcon + cnstr.num_cnstr;
      obj.nlcon_xind = [obj.nlcon_xind,{xind}];
    end
    
    
    
    function obj = addLinearConstraint(obj,cnstr,xind)
      % add a LinearConstraint to the program
      % @param cnstr     -- A LinearConstraint object
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the constraint. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_vars)';
      end
      xind = xind(:);
      if(~isa(cnstr,'LinearConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addLinearConstraint expects a LinearConstraint object');
      end
      obj.lcon = [obj.lcon,{cnstr}];
      
      cnstr_A = sparse(cnstr.iAfun,xind(cnstr.jAvar),cnstr.A_val,cnstr.num_cnstr,obj.num_vars,cnstr.nnz);
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
      obj.Ain_name = [obj.Ain_name;cnstr.name(cnstr.cin_idx)];
      obj.Aeq_name = [obj.Aeq_name;cnstr.name(cnstr.ceq_idx)];
      obj = obj.addLinearEqualityConstraints(cnstr_Aeq,cnstr_beq);
    end
    
    function obj = addBoundingBoxConstraint(obj,cnstr,xind)
      % add a BoundingBoxConstraint to the program
      % @param cnstr      -- A BoundingBoxConstraint
      % @param xind       -- Optional argument. x(xind) is the decision variables to be
      % set bounds
      if(nargin < 3)
        xind = (1:obj.num_vars)';
      end
      if(~isa(cnstr,'BoundingBoxConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addBoundingBoxConstraint expects a BoundingBoxConstraint object');
      end
      obj.bbcon = [obj.bbcon,{cnstr}];
      obj.x_lb(xind) = max([cnstr.lb obj.x_lb(xind)],[],2);
      obj.x_ub(xind) = min([cnstr.ub obj.x_ub(xind)],[],2);
    end
    
    function obj = addCost(obj,cnstr,xind)
      % Add a cost to the objective function
      % @param cnstr   -- A NonlinearConstraint or a LinearConstraint
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_vars)';
      end
      xind = xind(:);
      if(~isa(cnstr,'LinearConstraint') && ~isa(cnstr,'NonlinearConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addCost expects a LinearConstraint or NonlinearConstraint object');
      end
      if(isa(cnstr,'LinearConstraint'))
        if(cnstr.num_cnstr ~= 1)
          error('Drake:NonlinearProgramWConstraint:WrongCost','addCost only accept scalar function');
        end
        obj.cost = [obj.cost,{cnstr}];
        obj.cost_xind_cell = [obj.cost_xind_cell,{xind(cnstr.jAvar)}];
        obj.jFvar = unique([obj.jFvar;xind(cnstr.jAvar)]);
        obj.iFfun = ones(length(obj.jFvar),1);
      elseif(isa(cnstr,'NonlinearConstraint'))
        if(cnstr.num_cnstr ~= 1)
          error('Drake:NonlinearProgramWConstraint:WrongCost','addCost only accept scalar function');
        end
        obj.cost = [obj.cost,{cnstr}];
        obj.cost_xind_cell = [obj.cost_xind_cell,{xind(cnstr.jCvar)}];
        obj.jFvar = unique([obj.jFvar;xind(cnstr.jCvar)]);
        obj.iFfun = ones(length(obj.jFvar),1);
      end
    end
    
    function [g,h,dg,dh] = nonlinearConstraints(obj,x)
      f = zeros(obj.num_nlcon,1);
      G = zeros(obj.num_nlcon,obj.num_vars);
      f_count = 0;
      for i = 1:length(obj.nlcon)
        [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
          obj.nlcon{i}.eval(x(obj.nlcon_xind{i}));
        f(f_count+obj.nlcon{i}.ceq_idx) = f(f_count+obj.nlcon{i}.ceq_idx)-obj.nlcon{i}.ub(obj.nlcon{i}.ceq_idx);
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      g = f(obj.nlcon_ineq_idx);
      h = f(obj.nlcon_eq_idx);
      dg = G(obj.nlcon_ineq_idx,:);
      dh = G(obj.nlcon_eq_idx,:);
    end
    
    
    function [f,df] = objective(obj,x)
      f = 0;
      df = zeros(1,obj.num_vars);
      for i = 1:length(obj.cost)
        [fi,dfi] = obj.cost{i}.eval(x(obj.cost_xind_cell{i}));
        f = f+fi;
        df(obj.cost_xind_cell{i}) = df(obj.cost_xind_cell{i})+dfi;
      end
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      f = zeros(1+obj.num_nlcon,1);
      G = zeros(1+obj.num_nlcon,obj.num_vars);
      for i = 1:length(obj.cost)
        [fi,dfi] = obj.cost{i}.eval(x(obj.cost_xind_cell{i}));
        f(1) = f(1)+fi;
        G(1,obj.cost_xind_cell{i}) = G(1,obj.cost_xind_cell{i})+dfi;
      end
      f_count = 1;
      for i = 1:length(obj.nlcon)
        [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
          obj.nlcon{i}.eval(x(obj.nlcon_xind{i}));
        f(f_count+obj.nlcon{i}.ceq_idx) = f(f_count+obj.nlcon{i}.ceq_idx)-obj.nlcon{i}.ub(obj.nlcon{i}.ceq_idx);
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      f = [f(1);f(1+obj.nlcon_ineq_idx);f(1+obj.nlcon_eq_idx)];
      G = [G(1,:);G(1+obj.nlcon_ineq_idx,:);G(1+obj.nlcon_eq_idx,:)];
    end
    
    function obj = addDecisionVariable(obj,num_new_vars)
      % appending new decision variables to the end of the current decision variables
      % @param num_new_vars      -- An integer. The newly added decision variable is an
      % num_new_vars x 1 double vector.
      obj.num_vars = obj.num_vars+num_new_vars;
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
        xind = (1:obj.num_vars)';
      end
      xind = xind(:);
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