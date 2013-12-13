function [x,fval,info,active] = quadraticProgram(Q,f,Ain,bin,Aeq,beq,lb,ub,active,options)

% Quadratic Program attempts to provide a common interface to
% the wealth of QP solvers that we have kicking around.  It 
% attempts to solve the following formulation:
% 
%  minimize_x (1/2 * x'diag(Qdiag)'x + f'x)
%  subject to
%              Ain x <= bin
%              Aeq x = beq
%              lb<=x<=ub
%
% @option solver can be one of 'gurobi','fastQP','gurobi_mex'
% (coming soon: 'quadprog','cplex','mosek','snopt')
% @option x0 initial guess at solution
%

n = length(f);
if nargin<3 || isempty(Ain), Ain=zeros(0,n); end
if nargin<4, bin=[]; end
if nargin<5 || isempty(Aeq), Aeq=zeros(0,n); end
if nargin<6, beq=[]; end
if nargin<7, lb=[]; end
if nargin<8, ub=[]; end
if nargin<9, active=[]; end

if nargin<10, options=struct(); end
if ~isfield(options,'solver'), 
  % todo: check dependencies here and pick my favorite that is also installed
  options.solver = 'gurobi'; 
end

switch lower(options.solver)
  case 'quadprog'
    [x,fval,info] = quadprog(Q,f,Ain,bin,Aeq,beq,lb,ub);
    active=[];
    if (nargout>3) warning('active not implemented yet for quadprog (but should be trivial)'); end
      
  case 'gurobi'
    [x,info,active] = mygurobi(Q,f,Ain,bin,Aeq,beq,lb,ub,active);
    fval = nan;  % todo: implement this
    
  case 'gurobi_mex'
    checkDependency('gurobi');
    [x,info,active] = gurobiQPmex(Q,f,Aeq,beq,Ain,bin,lb,ub,active);
    fval = nan;  % todo: implement this

  case 'fastqp'
    if isempty(lb), lb=-inf + 0*f; end
    if isempty(ub), ub=inf + 0*f; end
    Ain_b = [Ain; -eye(length(lb)); eye(length(ub))];
    bin_b = [bin; -lb; ub];

    [x,info,active] = fastQPmex(Q,f,Aeq,beq,Ain_b,bin_b,active);
    fval = nan;  % todo: implement this
    
  otherwise
    error('Drake:QuadraticProgram:UnknownSolver',['The requested solver, ',options.solver,' is not known, or not currently supported']);
end

end



function [x,info,active] = mygurobi(Q,f,Ain,bin,Aeq,beq,lb,ub,active)

checkDependency('gurobi');

params.outputflag = 0; % not verbose
params.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
params.presolve = 0;
if params.method == 2
  params.bariterlimit = 20; % iteration limit
  params.barhomogeneous = 0; % 0 off, 1 on
  params.barconvtol = 5e-4;
end

if iscell(Q), 
  for i=1:length(Q), if isvector(Q{i}), Q{i} = diag(Q{i}); end, end
  Q = blkdiag(Q{:});
elseif isvector(Q), Q = diag(Q); end
model.Q = sparse(Q);
model.obj = f;
model.A = sparse([Aeq;Ain]);
model.rhs = [beq,bin];
model.sense = char([repmat('=',length(beq),1); repmat('<',length(bin),1)]);
if isempty(lb)
  model.lb = -inf + 0*f;
else
  model.lb = lb;
end
if isempty(ub)
  model.ub = inf + 0*f;
else
  model.ub = ub;
end

if ~isempty(model.A) && params.method==2 
  model.obj = 2*model.obj;   
  % according to the documentation, I should always need this ...
  % (they claim to optimize x'Qx + f'x), but it seems that they are off by
  % a factor of 2 when there are no constraints.
end


result = gurobi(model,params);

info = result.status;
x = result.x;

if isempty(model.A) && params.method==2
  x = x/2;   
  % according to the documentation, I should always need this ...
  % (they claim to optimize x'Qx + f'x), but it seems that they are off by
  % a factor of 2 when there are no constraints.
end

if (size(Ain,1)>0 || ~isempty(lb) || ~isempty(ub))
  % note: result.slack(for Ain indices) = bin-Ain*x
  active = find([result.slack(size(Aeq,1)+1:end); x-model.lb; model.ub-x]<1e-4);
else
  active=[];
end

end

% NOTEST