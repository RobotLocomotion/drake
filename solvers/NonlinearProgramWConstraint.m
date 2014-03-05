classdef NonlinearProgramWConstraint < NonlinearProgram
  % The constraint of this nonlinear program is specified using 'Constraint' class in
  % drake
  % @param nlcon     -- A cell array of NonlinearConstraint
  % @param lcon      -- A cell array of LinearConstraint
  % @param num_nlcon -- The total number of nonlinear constraints
  % @param num_lcon  -- The total number of linear constraints
  properties(SetAccess = protected)
    nlcon
    lcon
    num_nlcon
    num_lcon
  end
  
  properties(Access = protected)
    nlcon_xind % A cell array, nlcon_xind{i} is an int vector recording the indices of x that is used in evaluation the i'th NonlinearConstraint
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
    end
    
    function obj = addNonlinearConstraint(obj,cnstr,xind)
      % add a nonlinear constraint to the object, change the constraint evalation of the
      % program. 
      % @param cnstr     -- A NonlinearConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_decision_vars);
      end
      xind = xind(:);
      if(~isa(cnstr,'NonlinearConstraint'))
        error('Drake:NonlinearProgramWConstraint:UnsupportedConstraint','addNonlinearConstraint expects a NonlinearConstraint object');
      end
      obj.nlcon = [obj.nlcon,{cnstr}];
      obj.x_lb(xind) = max([obj.x_lb(xind) cnstr.x_lb],[],2);
      obj.x_ub(xind) = min([obj.x_ub(xind) cnstr.x_ub],[],2);
      
      obj.ceq = [obj.ceq;(cnstr.c_lb(cnstr.ceq_idx)+cnstr.c_ub(cnstr.ceq_idx))/2];
      obj.cin_ub = [obj.cin_ub;cnstr.c_ub(cnstr.cin_idx)];
      obj.cin_lb = [obj.cin_lb;cnstr.c_lb(cnstr.cin_idx)];
      obj.iGfun = [obj.iGfun;obj.num_nlcon + cnstr.iCfun];
      obj.jGvar = [obj.jGvar;xind(cnstr.jCvar)];
      obj.nlcon_ineq_idx = [obj.nlcon_ineq_idx;obj.num_nlcon+cnstr.cin_idx];
      obj.nlcon_eq_idx = [obj.nlcon_eq_idx;obj.num_nlcon+cnstr.ceq_idx];
      obj.num_nonlinear_inequality_constraints = obj.num_nonlinear_inequality_constraints + length(cnstr.cin_idx);
      obj.num_nonlinear_equality_constraints = obj.num_nonlinear_equality_constraints + length(cnstr.ceq_idx);
      obj.num_nlcon = obj.num_nlcon + cnstr.num_cnstr;
      obj.nlcon_xind = [obj.nlcon_xind,{xind}];
    end
    
    function [g,h,dg,dh] = nonlinearConstraints(obj,x)
      c = zeros(obj.num_nlcon,1);
      dc_val = zeros(length(obj.iGfun),1);
      nlcon_count = 0;
      dc_count = 0;
      for i = 1:length(obj.nlcon)
        [c(nlcon_count+(1:obj.nlcon{i}.num_cnstr)),dc_val(dc_count+(1:obj.nlcon{i}.nnz))]...
          = obj.nlcon{i}.evalSparse(x(obj.nlcon_xind{i}));
        nlcon_count = nlcon_count+obj.nlcon{i}.num_cnstr;
        dc_count = dc_count+obj.nlcon{i}.nnz;
      end
      dc = sparse(obj.iGfun,obj.jGvar,dc_val,obj.num_nlcon,obj.num_decision_vars);
      g = c(obj.nlcon_ineq_idx);
      h = c(obj.nlcon_eq_idx);
      dg = dc(obj.nlcon_ineq_idx,:);
      dh = dc(obj.nlcon_eq_idx,:);
    end
    
  end
end