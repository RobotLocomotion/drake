classdef QuadraticProgram < NonlinearProgram

% Provides a common interface to the wealth of QP solvers that we 
% have kicking around.  It attempts to solve the following formulation:
% 
%  minimize_x (1/2 * x'Q'x + f'x)
%  subject to
%              Ain x <= bin
%              Aeq x = beq
%              x_lb<=x<=x_ub
%
% @option solver can be one of 'gurobi','fastQP','gurobi_mex'
% (coming soon: 'quadprog','cplex','mosek','snopt')
% @retval x the optimal solution
% @retval fval the objective value at the optimal solution
% @retval info depends on the solver.  todo: make this consistent
% @retval active active set at termination of the algorithm
%

% todo: implement the following features:
% @option x0 initial guess at solution (allowed for some methods) 
% verbose output (especially for info ~= 1)
% custom options to pass through to solver
%

properties
  Q,f
end

methods
  function obj = QuadraticProgram(Q,f,Ain,bin,Aeq,beq,x_lb,x_ub)
    n = length(f);
    if nargin<3 || isempty(Ain), Ain=zeros(0,n); end
    if nargin<4, bin=[]; end
    if nargin<5 || isempty(Aeq), Aeq=zeros(0,n); end
    if nargin<6, beq=[]; end
    if nargin<7 || isempty(x_lb), x_lb=-inf(n,1); end
    if nargin<8 || isempty(x_ub), x_ub=inf(n,1); end

    obj = obj@NonlinearProgram(size(Q,1));
    sizecheck(Q,[obj.num_vars,obj.num_vars]);
    if(any(eig(Q)<0))
      error('Drake:QuadraticProgram:NegativeHessian','Q should be a positive-semidefinite matrix');
    end
    
    sizecheck(f,[obj.num_vars,1]);
    obj.Q = zeros(obj.num_vars,obj.num_vars);
    obj.f = zeros(obj.num_vars,1);
    obj = obj.addCost(QuadraticConstraint(-inf,inf,Q,f));
    obj = obj.addLinearConstraint(LinearConstraint(-inf(numel(bin),1),bin(:),Ain));
    obj = obj.addLinearConstraint(LinearConstraint(beq,beq,Aeq));
    obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x_lb,x_ub));
    obj.Q = Q;
    obj.f = f;
    obj = obj.setSolver('default');
    obj.solver_options.quadprog = optimset('Display','off');
  end

  function [x,objval,exitflag,active] = solve(obj,x0,active)
    if (nargin<2) x0 = randn(obj.num_vars,1); end
    if (nargin<3) active=[]; end
    
    switch lower(obj.solver)
      case 'quadprog'
        [x,objval,exitflag] = quadprog(obj.Q,obj.f,obj.Ain,obj.bin,obj.Aeq,obj.beq,obj.x_lb,obj.x_ub,[],obj.solver_options.quadprog);
        if (nargout>3)
          active=[];
          warning('active not implemented yet for quadprog (but should be trivial)');
        end
        
      case 'gurobi'
        [x,objval,exitflag,active] = gurobiQP(obj,active);
    
      case 'gurobi_mex'
        [x,exitflag,active] = gurobiQPmex(obj.Q,obj.f,obj.Ain,obj.bin,obj.Aeq,obj.beq,obj.x_lb,obj.x_ub,active);
        if (nargout>1)
          objval = .5*x'*obj.Q*x + obj.f'*x;  % todo: push this into mex?
        end
    
      case 'fastqp'
        if isempty(obj.x_lb), obj.x_lb=-inf + 0*obj.f; end
        if isempty(obj.x_ub), obj.x_ub=inf + 0*obj.f; end
        Ain_b = [obj.Ain; -eye(length(obj.x_lb)); eye(length(obj.x_ub))];
        bin_b = [obj.bin; -obj.x_lb; obj.x_ub];

        [x,exitflag,active] = fastQPmex(obj.Q,obj.f,Ain_b,bin_b,obj.Aeq,obj.beq,active);
        if (nargout>1)
          objval = .5*x'*obj.Q*x + obj.f'*x;  % todo: push this into mex?
        end
    
      otherwise
        [x,objval,exitflag] = solve@NonlinearProgram(obj,x0);
    end
  end

  function [x,objval,exitflag,execution_time] = compareSolvers(obj,x0,solvers)
    if nargin<2, x0 = randn(obj.num_vars,1); end
    if nargin<3, solvers = {'quadprog','gurobi','gurobi_mex','fastqp','snopt'}; end
    [x,objval,exitflag,execution_time] = compareSolvers@NonlinearProgram(obj,x0,solvers);
  end   
  
  function [f,df] = objectiveAndNonlinearConstraints(obj,x)
    f = .5*x'*obj.Q*x + obj.f'*x;
    df = x'*obj.Q + obj.f';
  end
  
  function [obj,cnstr_id] = addNonlinearConstraint(obj,cnstr,xind,data_ind)
    error('Drake:QuadraticProgram:UnsupportedConstraint','QuadraticProgram does not accept NonlinearConstraint');
  end

  function obj = addCost(obj,cnstr,xind,data_ind)
    % @param cnstr  A QuadraticConstraint
    if(~isa(cnstr,'QuadraticConstraint') && ~isa(cnstr,'LinearConstraint'))
      error('Drake:QuadraticProgram:UnsupportedConstraint','QuadraticProgram only accepts quadratic or linear cost');
    end
    if(nargin<3)
      xind = (1:obj.num_vars)';
    end
    if(nargin<4)
      data_ind = [];
    end
    obj = addCost@NonlinearProgram(obj,cnstr,xind,data_ind);
    if(isa(cnstr,'QuadraticConstraint'))
      obj.Q(xind,xind) = obj.Q(xind,xind)+cnstr.Q;
      if(any(eig(obj.Q)<0))
        error('Drake:QuadraticProgram:NegativeHessian','QuadraticProgram expects Hessian to be PSD');
      end
      obj.f(xind) = obj.f(xind)+cnstr.b;
    elseif(isa(cnstr,'LinearConstraint'))
      if(cnstr.num_cnstr~= 1)
        error('Drake:QuadraticProgram:UnsupportedConstraint','QuadraticProgram cannot accept LinearConstraint with multiple rows in its objective')
      end
      obj.f(xind) = obj.f(xind)+A';
    end
  end
  
  function obj = addDecisionVariable(obj,num_new_vars,var_name)
    error('Not implemented yet');
  end
  
  function obj = replaceCost(obj,cost,cost_idx,xind)
    error('Not implemented yet');
  end
  
  function obj = setSolver(obj,solver)
    if(strcmp(solver,'gurobi'))
      if(~checkDependency('gurobi'))
        error('Drake:QuadraticProgram:UnsupportedSolver','gurobi is not installed');
      end
      obj.solver = solver;
    elseif(strcmp(solver,'quadprog'))
      obj.solver = solver;
    elseif(strcmp(solver,'gurobi_mex'))
      if(~checkDependency('gurobi_mex'))
        error('Drake:UnsupportedSolver','gurobi_mex is not installed');
      end
      obj.solver = solver;
    elseif(strcmp(solver,'fastqp'))
      if(~checkDependency('fastqp'))
        error('Drake:UnsupportedSolver','fastqp is not installed');
      end
      obj.solver = solver;
    elseif(strcmp(solver,'default'))
      if checkDependency('gurobi')
        obj.solver = 'gurobi';
      elseif checkDependency('gurobi_mex')  % possible if gurobi was compiled, but addpath_gurobi isn't loaded
        obj.solver = 'gurobi_mex';
      elseif checkDependency('quadprog')
        obj.solver = 'quadprog';
      else
        obj = setSolver@NonlinearProgram(obj,'default');
      end
    else
      obj = setSolver@NonlinearProgram(obj,solver);
    end
  end
end

methods(Access=protected)
  function [x,objval,info,active] = gurobiQP(obj,active)
    
    params.outputflag = 0; % not verbose
    params.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
    params.presolve = 0;
    if params.method == 2
      params.bariterlimit = 20; % iteration limit
      params.barhomogeneous = 0; % 0 off, 1 on
      params.barconvtol = 5e-4;
    end
    
    if iscell(obj.Q),
      for i=1:length(obj.Q), if isvector(obj.Q{i}), obj.Q{i} = diag(obj.Q{i}); end, end
      obj.Q = blkdiag(obj.Q{:});
    elseif isvector(obj.Q), obj.Q = diag(obj.Q); end
    model.Q = sparse(obj.Q);
    model.obj = full(obj.f);
    model.A = sparse([obj.Aeq;obj.Ain]);
    model.rhs = full([obj.beq;obj.bin]);
    model.sense = char([repmat('=',length(obj.beq),1); repmat('<',length(obj.bin),1)]);
    if isempty(obj.x_lb)
      model.lb = -inf + 0*obj.f;
    else
      model.lb = obj.x_lb;
    end
    if isempty(obj.x_ub)
      model.ub = inf + 0*obj.f;
    else
      model.ub = obj.x_ub;
    end
    
    if params.method==2
      model.Q = .5*model.Q;
      % according to the documentation, I should always need this ...
      % (they claim to optimize x'Qx + f'x), but it seems that they are off by
      % a factor of 2 %% when there are no constraints.
    end
    
    result = gurobi(model,params);
    
    info = strcmp(result.status,'OPTIMAL');
    x = result.x;
    objval = result.objval;
    
    if (size(obj.Ain,1)>0 || ~isempty(obj.x_lb) || ~isempty(obj.x_ub))
      % note: result.slack(for Ain indices) = bin-Ain*x
      active = find([result.slack(size(obj.Aeq,1)+1:end); x-model.lb; model.ub-x]<1e-4);
    else
      active=[];
    end
    
  end
end

end
