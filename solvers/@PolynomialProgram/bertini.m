function [x,objval,exitflag] = bertini(obj,options)
% solve for KKT stationary and complementarity
% conditions (+ primal feasibility of the
% equality constraints) and return solutions
% w/ dual feasibility (and primal feasibility
% of inequality constraints).
%  http://en.wikipedia.org/wiki/Karush%E2%80%93Kuhn%E2%80%93Tucker_conditions

checkDependency('bertini');

eq = diff(obj.poly_objective, obj.decision_vars)'; % stationarity
vars = obj.decision_vars;

if ~isempty(obj.poly_equality_constraints)
  lambda = msspoly('l',length(obj.poly_equality_constraints));
  eq(1:obj.num_vars) = eq(1:obj.num_vars) + (lambda'*diff(obj.poly_equality_constraints, obj.decision_vars))';
  eq = vertcat(eq, obj.poly_equality_constraints); % primal feasibility
  vars = vertcat(vars, lambda);
end

if ~isempty(obj.poly_inequality_constraints)
  mu = msspoly('u',length(obj.poly_inequality_constraints));
  eq(1:obj.num_vars) = eq(1:obj.num_vars) + (mu'*diff(obj.poly_inequality_constraints, obj.decision_vars))';
  eq = vertcat(eq,mu.*obj.poly_inequality_constraints); % complementarity
  mu_indices = length(vars) + 1:length(obj.poly_inequality_constraints);
  vars = vertcat(vars, mu);
end

if ~isempty(obj.Ain) || ~isempty(obj.Aeq) || any(~isinf(obj.x_lb)) || any(~isinf(obj.x_ub))
  error('forgot to implement these on the first pass (but will be trivial)');
end

eq = clean(eq);
% convert to syms.  wish i didn't have to do this.  :)
symbolic_vars = sym('v',size(vars));
symbolic_vars = sym(symbolic_vars,'real');
symbolic_eq = msspoly2sym(vars,symbolic_vars,eq);

% todo: extract homogenous variable classes?

options.parameter = [];
options.useRegen = 1;
bertini_job = bertini(symbolic_eq,options);

sol = solve(bertini_job);
sol = get_values(bertini_job,sol,symbolic_vars);
sol = get_real(bertini_job,sol);
sol = sol(1:length(obj.decision_vars),:);

if ~isempty(obj.poly_inequality_constraints)
  sol = sol(:,all(sol(mu_indices,:)>=0)); % dual feasibility

  nonlinear_constraint_vals = double(msubs(obj.poly_inequality_constraints,obj.decision_vars,sol));
  sol = sol(:,all(nonlinear_constraint_vals<=0)); % primal feasibility
end

vals = double(msubs(obj.poly_objective,obj.decision_vars,sol));
[objval,optimal_solution_index] = min(vals); objval=full(objval);
x = sol(:,optimal_solution_index);
exitflag = 1;  % todo: do this better

end
