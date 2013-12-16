classdef PolynomialProgram < NonlinearProgram
% Provides a common interface for polynomial optimization.
%  (note: should perhaps be called a SpotPolynomialProgram, to parallel
%  PolynomialSystem/SpotPolynomialSystem)
%  
% Using x to denote the decision variables, attempts to solve the problem
%
%   minimize_x objective(x)
%   subject to 
%         equality_constraints(x) = 0 
%         inequality_constraint(x) <= 0
%
% @option solver can be 'gloptipoly','snopt' (more coming soon)
% @option x0 initial guess

% todo: implement other solution techniques:
%   bertini (via kkt)
%   ... my pcpo idea?  
  
  properties
    decision_vars     % simple msspoly description of x
    objective              % msspoly
    equality_constraints   % msspoly
    inequality_constraints % msspoly
  end

  methods
    function obj = PolynomialProgram(decision_vars,objective,equality_constraints,inequality_constraints)
      typecheck(decision_vars,'msspoly');
      if ~issimple(decision_vars),
        error('decision_vars must be a simple msspoly');
      end
      typecheck(objective,'msspoly');
      if length(objective)~=1
        error('objective must be a scalar msspoly');
      end
      
      if nargin<3, equality_constraints = []; end
      if nargin<4, inequality_constraints = []; end
      if nargin<5, options=struct(); end
      if ~isfield(options,'solver'),
        % todo: check dependencies here and pick my favorite that is also installed
        options.solver = 'bertini';
      end
      
      if ~isfunction(objective,decision_vars)
        error('objective function must only depend on the decision variables');
      end
      if ~isempty(equality_constraints)
        sizecheck(equality_constraints,'colvec');
        if ~isfunction(equality_constraints,decision_vars)
          error('equality constraints must only depend on the decision variables');
        end
      end
      if ~isempty(inequality_constraints)
        sizecheck(inequality_constraints,'colvec');
        if ~isfunction(inequality_constraints,decision_vars)
          error('equality constraints must only depend on the decision variables');
        end
      end

      obj = obj@NonlinearProgram(length(decision_vars),length(equality_constraints),length(inequality_constraints));
      obj.decision_vars = decision_vars;
      obj.objective = objective;
      obj.equality_constraints = equality_constraints;
      obj.inequality_constraints = inequality_constraints;
    end
    
    function [x,objval,exitflag] = solve(obj,x0)
      switch lower(obj.solver)
        case 'bertini'
          [x,objval,exitflag] = bertini(obj);
        otherwise 
          [x,objval,exitflag] = solve@NonlinearProgram(obj,x0);
      end
    end

    function [x,objval,exitflag,execution_time] = compareSolvers(obj,x0,solvers)
      if nargin<2, x0 = randn(obj.num_decision_vars,1); end
      if nargin<3, solvers = {'bertini','snopt','fmincon'}; end
      [x,objval,exitflag,execution_time] = compareSolvers@NonlinearProgram(obj,x0,solvers);
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      poly_f = [obj.objective; obj.equality_constraints; obj.inequality_constraints];
      poly_G = diff(poly_f,obj.decision_vars); % note: i could do this just once
      
      f = double(subs(poly_f,obj.decision_vars,x));
      G = double(subs(poly_G,obj.decision_vars,x));
    end
    
    function [x,objval,exitflag] = bertini(obj,options)
      % solve for KKT stationary and complementarity 
      % conditions (+ primal feasibility of the 
      % equality constraints) and return solutions
      % w/ dual feasibility (and primal feasibility 
      % of inequality constraints).
      %  http://en.wikipedia.org/wiki/Karush%E2%80%93Kuhn%E2%80%93Tucker_conditions

      checkDependency('bertini');

      eq = diff(obj.objective, obj.decision_vars); % stationarity
      vars = obj.decision_vars;
      
      if ~isempty(obj.equality_constraints)
        lambda = msspoly('l',length(obj.equality_constraints));
        eq(1) = eq(1) + lambda'*diff(obj.equality_constraints, obj.decision_vars);
        eq = vertcat(eq, obj.equality_constraints); % primal feasibility
        vars = vertcat(vars, lambda);
      end        

      if ~isempty(obj.inequality_constraints)
        mu = msspoly('u',length(obj.inequality_constraints));
        eq(1) = eq(1) + mu'*diff(obj.inequality_constraints, obj.decision_vars);
        eq = vertcat(eq,mu.*obj.inequality_constraints); % complementarity
        mu_indices = length(vars) + 1:length(obj.inequality_constraints);
        vars = vertcat(vars, mu);
      end
      
      % convert to syms.  wish i didn't have to do this.  :)
      symbolic_vars = sym('v',size(vars));
      symbolic_vars = sym(symbolic_vars,'real');
      symbolic_eq = msspoly2sym(vars,symbolic_vars,eq);
      
      % todo: extract homogenous variable classes?
      
      options.parameter = [];
      bertini_job = bertini(symbolic_eq,options);
      
      sol = solve(bertini_job);
      sol = get_real(bertini_job,sol);
      
      if ~isempty(obj.inequality_constraints)
        sol = sol(:,all(sol(mu_indices,:)>=0)); % dual feasibility
        
        nonlinear_constraint_vals = double(msubs(obj.inequality_constraints,obj.decision_vars,sol));
        sol = sol(:,all(nonlinear_constraint_vals<=0)); % primal feasibility
      end
      
      vals = double(msubs(obj.objective,obj.decision_vars,sol));
      [objval,optimal_solution_index] = min(vals); objval=full(objval);
      x = sol(:,optimal_solution_index);
      exitflag = 1;  % todo: do this better
    end
  end
  
end