function V = regionOfAttraction(sys,x0,V0,options)
% estimates the region of attraction, defined as the interior region for
% the level-set V<=1 surrounding the origin.  V0 is the initial guess for V.  
% Both V0 and V are specified relative to x0 (e.g. V(0) is the value of the
% Lyapunov function at x0).  

if (nargin<4) options=struct(); end
if (~isfield(options,'method')) options.method='pablo'; end

if (~isCT(sys)) error('only handle CT case so far'); end
if (~isTI(sys)) error('only for TI systems so far'); end

num_x = sys.getNumStates();
num_u = sys.getNumInputs();
if (nargin<2 || isempty(x0))
  x0=zeros(num_x,1);
else
  typecheck(x0,'double');
  sizecheck(x0,[num_x,1]);
end

% check fixed point
% check Hessian Vdot at origin, to make sure it's negative def. 

x = sys.p_x;
% f is time-invariant zero-input dynamics relative to x0
f = subss(sys.p_dynamics,[sys.p_t;x;sys.p_u],[0;x + x0;zeros(num_u,1)]);
A = doubleSafe(subs(diff(f,x),x,zeros(num_x,1)));

if (nargin<3 || isempty(V0)) 
  Q = eye(sys.num_x);  % todo: take this as a parameter?
  P = lyap(A',Q);
  V = x'*P*x;
elseif (isnumeric(V0) && ismatrix(V0) && all(size(V0)==[num_x,num_x]))  % then use this quadratic form
  V = x'*V0*x;
else
  V = V0;
end

typecheck(V,'msspoly');
if any([V.m V.n]~=1) error('V0 must be a scalar msspoly'); end

%% perform balancing
S = .5*doubleSafe(subs(diff(diff(V,x)',x),x,zeros(size(x0))));  % extract Hessian
[T,D] = balanceQuadForm(S,(S*A+A'*S));
V=subss(V,x,T*x);  
f=inv(T)*subss(f,x,T*x);

%% compute Vdot
Vdot = diff(V,x)*f;

if (~isfield(options,'monom_order'))
  options.monom_order = deg(Vdot,x);  % just a guess
end

%% compute level set
switch (options.method)
  case 'pablo'
    V = pabloMethod(x,V,Vdot,options);
  case 'pablo_yalmip'
    V = pabloMethodYalmip(x,V,Vdot,options);
  case 'binary'
    V = rhoLineSearch(x,V,Vdot,options);
  otherwise
    error(['don''t know method: ', options.method]);
end

%% undo balancing
V = subss(V,x,inv(T)*x);

end


function V = rhoLineSearch(x,V,Vdot,options)
  prog = mssprog;
  Lmonom = monomials(x,0:options.monom_order);
  [prog,l] = new(prog,length(Lmonom),'free');
  L = l'*Lmonom;
  prog.sos = L;
  
  %% bracket the solution
  rhomin=0; rhomax=1;
  while ( checkRho(rhomax, x,V,Vdot,prog,L) > 0 )
    rhomin = rhomax;
    rhomax = 1.2*rhomax;
  end
  
  %% now do binary search (mark's version might be better here)
  rho = fzero(@(rho) checkRho(rho, x,V,Vdot,prog,L),[rhomin rhomax])

  V = V/rho;
end

function [slack,info] = checkRho(rho, x,V,Vdot,prog,L)
  [prog,slack] = new(prog,1,'free');
  
  prog.sos = -Vdot + L*(V - rho) - slack*V;
  
  [prog,info] = sedumi(prog,-slack,0);
  if (info.numerr>1)
    error('sedumi had numerical issues.  try reducing the order of the lagrange multipliers');
  end
  if (info.pinf || info.dinf)
    error('problem looks infeasible.  try increasing the order of the lagrange multipliers');
  end
  slack = doubleSafe(prog(slack));
end

function V = pabloMethod(x,V,Vdot,options)
  prog = mssprog;
  Lmonom = monomials(x,0:options.monom_order);

  rho = msspoly('r');
  prog.free = rho;
    
  [prog,l] = new(prog,length(Lmonom),'free');
  L = l'*Lmonom;
  
  prog.sos = (x'*x)*(V - rho) +  L*Vdot;
  [prog,info] = sedumi(prog,-rho,0); %1,struct());
  if (info.numerr>1)
    error('sedumi had numerical issues.  try reducing the order of the lagrange multipliers');
  end
  if (info.pinf || info.dinf)
    error('problem looks infeasible.  try increasing the order of the lagrange multipliers');
  end
  
  rho = doubleSafe(prog(rho))
  if (rho<=0) error('optimization failed'); end

  V = V/rho;
end

function V=pabloMethodYalmip(x,V,Vdot,options)
  checkDependency('yalmip_enabled');

  % flip to sdpvars
  xs = sdpvar(length(x),1);
  Vs = msspoly2sdpvar(x,xs,V);
  Vdots = msspoly2sdpvar(x,xs,Vdot);

  rho = sdpvar(1);
  [L,c]=polynomial(xs,options.monom_order);

  cnst = sos( (xs'*xs)*(Vs-rho) + L*Vdots );
  
  diagnostics=solvesos(cnst,-rho,[],[rho;c]);
  if (diagnostics.problem~=0)
    error(yalmiperror(diagnostics.problem));
  end
  rho = double(rho)
  
  V = V/rho;
end

function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

