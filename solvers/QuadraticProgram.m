classdef QuadraticProgram < NonlinearProgram

% Provides a common interface to the wealth of QP solvers that we 
% have kicking around.  It attempts to solve the following formulation:
% 
%  minimize_x (1/2 * x'diag(Qdiag)'x + f'x)
%  subject to
%              Ain x <= bin
%              Aeq x = beq
%              lb<=x<=ub
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
  function obj = QuadraticProgram(Q,f,Ain,bin,Aeq,beq,lb,ub)
    n = length(f);
    if nargin<3 || isempty(Ain), Ain=zeros(0,n); end
    if nargin<4, bin=[]; end
    if nargin<5 || isempty(Aeq), Aeq=zeros(0,n); end
    if nargin<6, beq=[]; end
    if nargin<7, lb=[]; end
    if nargin<8, ub=[]; end

    obj = obj@NonlinearProgram(size(Q,1),0,0);
    obj.Q = Q;
    obj.f = f;
    obj.Ain = Ain;
    obj.bin = bin;
    obj.Aeq = Aeq;
    obj.beq = beq;
    obj.lb = lb;
    obj.ub = ub;
    
  % todo: check dependencies here and pick my favorite that is also installed
    obj.solver = 'gurobi';
    obj.default_options.quadprog = optimset('Display','off');
  end

  function [x,objval,exitflag,active] = solve(obj,x0,active)
    if (nargin<2) x0 = randn(obj.num_decision_vars,1); end
    if (nargin<3) active=[]; end
    
    switch lower(obj.solver)
      case 'quadprog'
        [x,objval,exitflag] = quadprog(obj.Q,obj.f,obj.Ain,obj.bin,obj.Aeq,obj.beq,obj.lb,obj.ub,[],obj.default_options.quadprog);
        if (nargout>3)
          active=[];
          warning('active not implemented yet for quadprog (but should be trivial)');
        end
        
      case 'gurobi'
        checkDependency('gurobi');
        [x,objval,exitflag,active] = gurobi(obj,active);
    
      case 'gurobi_mex'
        checkDependency('gurobi_mex');
        [x,exitflag,active] = gurobiQPmex(obj.Q,obj.f,obj.Ain,obj.bin,obj.Aeq,obj.beq,obj.lb,obj.ub,active);
        if (nargout>1)
          objval = .5*x'*obj.Q*x + obj.f'*x;  % todo: push this into mex?
        end
    
      case 'fastqp'
        checkDependency('gurobi_mex');
        if isempty(obj.lb), obj.lb=-inf + 0*obj.f; end
        if isempty(obj.ub), obj.ub=inf + 0*obj.f; end
        Ain_b = [obj.Ain; -eye(length(obj.lb)); eye(length(obj.ub))];
        bin_b = [obj.bin; -obj.lb; obj.ub];

        [x,exitflag,active] = fastQPmex(obj.Q,obj.f,Ain_b,bin_b,obj.Aeq,obj.beq,active);
        if (nargout>1)
          objval = .5*x'*obj.Q*x + obj.f'*x;  % todo: push this into mex?
        end
    
      otherwise
        [x,objval,exitflag] = solve@NonlinearProgram(obj,x0);
    end
  end

  function [x,objval,exitflag,execution_time] = compareSolvers(obj,x0,solvers)
    if nargin<2, x0 = randn(obj.num_decision_vars,1); end
    if nargin<3, solvers = {'quadprog','gurobi','gurobi_mex','fastqp','snopt'}; end
    [x,objval,exitflag,execution_time] = compareSolvers@NonlinearProgram(obj,x0,solvers);
  end  
  
  function [f,df] = objectiveAndNonlinearConstraints(obj,x)
    f = .5*x'*obj.Q*x + obj.f'*x;
    df = x'*obj.Q + obj.f';
  end
  
  function [x,objval,info,active] = gurobi(obj,active)

    checkDependency('gurobi');
    
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
    model.obj = obj.f;
    model.A = sparse([obj.Aeq;obj.Ain]);
    model.rhs = [obj.beq,obj.bin];
    model.sense = char([repmat('=',length(obj.beq),1); repmat('<',length(obj.bin),1)]);
    if isempty(obj.lb)
      model.lb = -inf + 0*obj.f;
    else
      model.lb = obj.lb;
    end
    if isempty(obj.ub)
      model.ub = inf + 0*obj.f;
    else
      model.ub = obj.ub;
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
    
    if (size(obj.Ain,1)>0 || ~isempty(obj.lb) || ~isempty(obj.ub))
      % note: result.slack(for Ain indices) = bin-Ain*x
      active = find([result.slack(size(obj.Aeq,1)+1:end); x-model.lb; model.ub-x]<1e-4);
    else
      active=[];
    end
    
  end

end

end
