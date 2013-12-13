function [x,objval,exitflag] = polynomialOptimization(decision_variables,objective,equality_constraints,inequality_constraints,options)

% Wrapper method to provide a common interface for polynomial optimization.
%  
% Using x to denote the decision variables, attempts to solve the problem
%
%   minimize_x objective(x)
%   subject to 
%         equality_constraints(x) = 0 
%         inequality_constraint(x) >= 0
%
% @param decision_variables a simple msspoly description of x
% @param objective an msspoly
% @option solver can be 'gloptipoly','snopt' (more coming soon)
% @option x0 initial guess

% todo: implement other solution techniques:
%   bertini (via kkt)
%   ... my pcpo idea?

typecheck(decision_variables,'msspoly');
if ~issimple(decision_variables),
  error('decision_variables must be a simple msspoly');
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
  options.solver = 'snopt'; 
end

if ~isfunction(objective,decision_variables)
  error('objective function must only depend on the decision variables');
end
if ~isempty(equality_constraints)
  sizecheck(equality_constraints,'colvec');
  if ~isfunction(equality_constraints,decision_variables)
    error('equality constraints must only depend on the decision variables');
  end
end
if ~isempty(inequality_constraints) 
  sizecheck(inequality_constraints,'colvec');
  if ~isfunction(inequality_constraints,decision_variables)
    error('equality constraints must only depend on the decision variables');
  end
end

N = length(decision_variables);
Neq = length(equality_constraints);
Nineq = length(inequality_constraints);

switch lower(options.solver)
  case 'snopt'
    checkDependency('snopt');

    if ~isfield(options,'x0')
      options.x0 = zeros(N,1);
    end
    
    poly_f = [objective; equality_constraints; inequality_constraints];
    poly_G = diff(poly_f,decision_variables);

    % todo: tell snopt explicitly about linear terms and sparse gradients
    
    global SNOPT_USERFUN;
    SNOPT_USERFUN = @(x)snopt_userfun(x,decision_variables,poly_f,poly_G);
    
    [x,objval,exitflag] = snopt(options.x0,-inf(N,1),inf(N,1),[-inf;zeros(Neq+Nineq,1)],[inf;zeros(Neq,1);inf(Nineq,1)],'snoptUserfun');%,0,1,A,iAfun,jAvar,iGfun,jGvar);
    
  otherwise
    error('Drake:QuadraticProgram:UnknownSolver',['The requested solver, ',options.solver,' is not known, or not currently supported']);
end


end

function [f,G] = snopt_userfun(x,decision_variables,poly_f,poly_G)
  f = double(subs(poly_f,decision_variables,x));
  G = double(subs(poly_G,decision_variables,x));
end
