function [x,info,active] = mygurobi(Q,f,Aeq,beq,Ain,bin,lb,ub,active)

n = length(f);
if nargin<3, Aeq=zeros(0,n); end
if nargin<4, beq=[]; end
if nargin<5, Ain=zeros(0,n); end
if nargin<6, bin=[]; end
if nargin<7, lb=[]; end
if nargin<8, ub=[]; end
if nargin<9, active=[]; end

params.outputflag = 0; % not verbose
params.method = -1; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
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
model.sense = [repmat('=',length(beq),1); repmat('<',length(bin),1)];
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

result = gurobi(model,params);

info = result.status;
x = result.x;
if (size(Ain,1)>0)
  active = find(abs(result.slack(size(Aeq,1)+1:end))<1e-6);
else
  active=[];
end