function [x,info,active] = mygurobi(Q,f,Aeq,beq,Ain,bin,lb,ub,active)

%min 1/2 * x'diag(Qdiag)'x + f'x s.t A x = b, Ain x <= bin, 1b<=x<=ub

n = length(f);
if nargin<3 || isempty(Aeq), Aeq=zeros(0,n); end
if nargin<4, beq=[]; end
if nargin<5 || isempty(Ain), Ain=zeros(0,n); end
if nargin<6, bin=[]; end
if nargin<7, lb=[]; end
if nargin<8, ub=[]; end
if nargin<9, active=[]; end

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

if (size(Ain,1)>0)
  active = find(abs(result.slack(size(Aeq,1)+1:end))<1e-6);
else
  active=[];
end