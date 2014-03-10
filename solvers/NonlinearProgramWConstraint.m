classdef NonlinearProgramWConstraint < NonlinearProgram
  % The constraint of this nonlinear program is specified using 'Constraint' class in
  % drake
  % @param nlcon     -- A cell array of NonlinearConstraint
  % @param lcon      -- A cell array of LinearConstraint
  % @param num_nlcon -- The total number of nonlinear constraints
  % @param num_lcon  -- The total number of linear constraints
  % @param cost      -- A cell array of NonlinearConstraint or LinearConstraint.
  
  properties(SetAccess = protected)
    nlcon
    lcon
    num_nlcon
    num_lcon
    cost
  end
  
  properties(Access = private)
    iCinfun; % The row index of the non-zero entries in the nonlinear inequality constraints gradients
    jCinvar; % The column index of the non-zero entries in the nonlinear inequality constraitns gradient
    iCeqfun; % The row index of the non-zero entries in the nonlinear equality constraints gradients
    jCeqvar; % The column index of the non-zero entries in the nonlinear inequality constraints gradients
    iCfun; % The row index of the non-zero entries in the nonlinear constraints gradients
    jCvar; % The column index of the non-zero entries in the nonlinear constraints gradients
    iFfun; % The row index of the non-zero entries in the objective
    jFvar; % The column index of non-zero entries in the objective
  end
  
  properties(Access = protected)
    nlcon_xind % A cell array, nlcon_xind{i} is an int vector recording the indices of x that is used in evaluation the i'th NonlinearConstraint
    cost_xind_cell % A cell array, cost_xind{i} is an int vector recording the indices of x that is used in evaluating obj.cost{i}

    nlcon_ineq_idx % row index of nonlinear inequality constraint
    nlcon_eq_idx % row index of nonlinear equality constraint
  
  end
  
  methods
    function obj = NonlinearProgramWConstraint(num_vars)
      % @param num_vars     -- The number of decision variables
      obj = obj@NonlinearProgram(num_vars,0,0);
      obj.nlcon = {};
      obj.lcon = {};
      obj.num_nlcon = 0;
      obj.num_lcon = 0;
      obj.nlcon_xind = {};
      obj.nlcon_ineq_idx = [];
      obj.nlcon_eq_idx = [];
      obj.iCinfun = [];
      obj.jCinvar = [];
      obj.iCeqfun = [];
      obj.jCeqvar = [];
      obj.iCfun = [];
      obj.jCvar = [];
      obj.iFfun = [];
      obj.jFvar = [];
      obj.cost = {};
      obj.cost_xind_cell = {};
    end
    
    function obj = addNonlinearConstraint(obj,cnstr,xind)
      % add a NonlinearConstraint to the object, change the constraint evalation of the
      % program. 
      % @param cnstr     -- A NonlinearConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_vars);
      end
      xind = xind(:);
      if(~isa(cnstr,'NonlinearConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addNonlinearConstraint expects a NonlinearConstraint object');
      end
      obj.nlcon = [obj.nlcon,{cnstr}];
      
      obj.ceq = [obj.ceq;(cnstr.lb(cnstr.ceq_idx)+cnstr.ub(cnstr.ceq_idx))/2];
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
      obj.iCeqfun = [obj.iCeqfun;obj.num_ceq+inv_ceq_idx(cnstr.iCfun(Geq_idx))];
      obj.jCeqvar = [obj.jCeqvar;xind(cnstr.jCvar(Geq_idx))];
      obj.iCinfun = [obj.iCinfun;obj.num_cin+inv_cin_idx(cnstr.iCfun(Gin_idx))];
      obj.jCinvar = [obj.jCinvar;xind(cnstr.jCvar(Gin_idx))];
      obj.iCfun = [obj.iCinfun;obj.iCeqfun];
      obj.jCvar = [obj.jCinvar;obj.jCeqvar];
      obj.num_cin = obj.num_cin + length(cnstr.cin_idx);
      obj.num_ceq = obj.num_ceq + length(cnstr.ceq_idx);
      obj.num_nlcon = obj.num_nlcon + cnstr.num_cnstr;
      obj.nlcon_xind = [obj.nlcon_xind,{xind}];
      obj.iGfun = [obj.iFfun;1+obj.iCinfun;1+obj.num_cin+obj.iCeqfun]; 
      obj.jGvar = [obj.jFvar;obj.jCinvar;obj.jCeqvar];
    end
    
    
    
    function obj = addLinearConstraint(obj,cnstr,xind)
      % add a LinearConstraint to the program
      % @param cnstr     -- A LinearConstraint object
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the constraint. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_vars);
      end
      xind = xind(:);
      if(~isa(cnstr,'LinearConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addLinearConstraint expects a LinearConstraint object');
      end
      
      
      cnstr_A = sparse(cnstr.iAfun,xind(cnstr.jAvar),cnstr.A_val,cnstr.num_cnstr,obj.num_vars,cnstr.nnz);
      cnstr_beq = (cnstr.lb(cnstr.ceq_idx)+cnstr.ub(cnstr.ceq_idx))/2;
      cnstr_Aeq = cnstr_A(cnstr.ceq_idx,:);
      cnstr_Ain = cnstr_A(cnstr.cin_idx,:);
      cnstr_bin_lb = cnstr.lb(cnstr.cin_idx);
      cnstr_bin_ub = cnstr.ub(cnstr.cin_idx);
      obj = obj.addLinearInequalityConstraints(cnstr_Ain,cnstr_bin_lb,cnstr_bin_ub);
      obj = obj.addLinearEqualityConstraints(cnstr_Aeq,cnstr_beq);
    end
    
    function obj = addBoundingBoxConstraint(obj,cnstr,xind)
      % add a BoundingBoxConstraint to the program
      % @param cnstr      -- A BoundingBoxConstraint
      % @param xind       -- Optional argument. x(xind) is the decision variables to be
      % set bounds
      if(nargin < 3)
        xind = (1:obj.num_vars);
      end
      if(~isa(cnstr,'BoundingBoxConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addBoundingBoxConstraint expects a BoundingBoxConstraint object');
      end
      obj.x_lb(xind) = max([cnstr.lb obj.x_lb(xind)],[],2);
      obj.x_ub(xind) = min([cnstr.ub obj.x_ub(xind)],[],2);
    end
    
    function obj = addCost(obj,cnstr,xind)
      % Add a cost to the objective function
      % @param cnstr   -- A NonlinearConstraint or a LinearConstraint
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_vars);
      end
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
      elseif(isa(cnstr,'NonlinearConstraint'))
        if(cnstr.num_cnstr ~= 1)
          error('Drake:NonlinearProgramWConstraint:WrongCost','addCost only accept scalar function');
        end
        obj.cost = [obj.cost,{cnstr}];
        obj.cost_xind_cell = [obj.cost_xind_cell,{xind(cnstr.jCvar)}];
        obj.jFvar = unique([obj.jFvar;xind(cnstr.jCvar)]);
      end
      obj.iFfun = ones(length(obj.jFvar),1);
      obj.iGfun = [obj.iFfun;1+obj.iCfun];
      obj.jGvar = [obj.jFvar;obj.jCvar];
    end
    
    function [g,h,dg,dh] = nonlinearConstraints(obj,x)
      c = zeros(obj.num_nlcon,1);
      dc_val = zeros(length(obj.iCfun),1);
      nlcon_count = 0;
      dc_count = 0;
      for i = 1:length(obj.nlcon)
        [c(nlcon_count+(1:obj.nlcon{i}.num_cnstr)),dc_val(dc_count+(1:obj.nlcon{i}.nnz))]...
          = obj.nlcon{i}.evalSparse(x(obj.nlcon_xind{i}));
        nlcon_count = nlcon_count+obj.nlcon{i}.num_cnstr;
        dc_count = dc_count+obj.nlcon{i}.nnz;
      end
      dc = sparse(obj.iCfun,obj.jCvar,dc_val,obj.num_nlcon,obj.num_vars);
      g = c(obj.nlcon_ineq_idx);
      h = c(obj.nlcon_eq_idx);
      dg = dc(obj.nlcon_ineq_idx,:);
      dh = dc(obj.nlcon_eq_idx,:);
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
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      f = [f(1);f(1+obj.nlcon_ineq_idx);f(1+obj.nlcon_eq_idx)];
      G = [G(1,:);G(1+obj.nlcon_ineq_idx,:);G(1+obj.nlcon_eq_idx,:)];
    end
  end
end