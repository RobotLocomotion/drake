function V = regionOfAttraction(sys,x0,V0,options)
% estimates the region of attraction, defined as the interior region for
% the level-set V<=1 surrounding x0. 

if (nargin<4) options=struct(); end
if (~isfield(options,'method')) options.method='pablo'; end
if (~isfield(options,'degV')) options.degV = 4; end
if (~isfield(options,'max_iterations')) options.max_iterations=10; end
if (~isfield(options,'converged_tol')) options.converged_tol=.01; end

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
else % take the V0 candidate, but center it around x0
  V = subss(V0,x,x+x0);
end

typecheck(V,'msspoly');
if any([V.m V.n]~=1) error('V0 must be a scalar msspoly'); end

if (strcmp(options.method,'bilinear'))
  V0 = V;
  rho = 1;  % todo: consider running pablo's method (always) first, to get a feasible initial guess

  if (~isfield(options,'degL1'))
    options.degL1 = options.degV-1 + deg(f,x);  % just a guess
  end
  if (~isfield(options,'degL2'))
    options.degL2 = options.degL1;
  end

  %% perform balancing (just once)
  S = .5*doubleSafe(subs(diff(diff(V,x)',x),x,zeros(size(x0))));  % extract Hessian
  [T,D] = balanceQuadForm(S,(S*A+A'*S));
%  T=eye(num_x);
  L1monom = monomials(x,0:options.degL1);
  L2monom = monomials(x,0:options.degL2);
  Vmonom = monomials(x,0:options.degV);
  
  vol=0;
  for iter=1:options.max_iterations
    last_vol = vol;
    
    % balance on every iteration (since V and Vdot are changing):
%    S1=.5*doubleSafe(subs(diff(diff(V,x)',x),x,0*x));
%    S2 = S1*A+A'*S1;
%%    Vdot = diff(V,x)*f;
%%    S2=.5*doubleSafe(subs(diff(diff(Vdot,x)',x),x,0*x));
%    [T,D] = balanceQuadForm(S1,S2);
%     L1monom = balanced_gaussian_orthogonal_basis(x,options.degL1,inv(D),D);
%     L2monom = balanced_gaussian_orthogonal_basis(x,options.degL2,inv(D),D);
%     Vmonom = balanced_gaussian_orthogonal_basis(x,options.degV,inv(D),D);
    Ti = inv(T);
    Vbal=subss(V,x,T*x);
    V0bal=subss(V0,x,T*x);
    fbal=Ti*subss(f,x,T*x);
    
    [L1,sigma1] = findL1(x,fbal,Vbal,L1monom,options);
    L2 = findL2(x,Vbal,V0bal,rho,L2monom,options);
    [Vbal,rho] = optimizeV(x,fbal,L1,L2,V0bal,sigma1,Vmonom,options);
    vol = rho
    
    % undo balancing (for the next iteration, or if i'm done)
    V = subss(Vbal,x,Ti*x);
    plotFunnel(V,0*x0); plotFunnel(V0/rho,0*x0,[],struct('color',[.9 .3 .2])); drawnow;
    
    % check for convergence
    if ((vol - last_vol) < options.converged_tol*last_vol)
      break;
    end
  end
  
else

  %% perform balancing (just once)
  S = .5*doubleSafe(subs(diff(diff(V,x)',x),x,zeros(size(x0))));  % extract Hessian
  [T,D] = balanceQuadForm(S,(S*A+A'*S));
  V=subss(V,x,T*x);
  f=inv(T)*subss(f,x,T*x);
  
  %% compute Vdot
  Vdot = diff(V,x)*f;

  if (~isfield(options,'degL1'))
    options.degL1 = deg(Vdot,x);  % just a guess
  end
  
  %% compute level set
  switch (options.method)
    case 'pablo'
      V = pabloMethod(x,V,Vdot,options);
    case 'pablo_yalmip'
      V = pabloMethodYalmip(x,V,Vdot,options);
    case 'binary'
      V = rhoLineSearch(x,V,Vdot,options);
    % note: bilinear implemented above
    otherwise
      error(['don''t know method: ', options.method]);
  end
  
  %% undo balancing
  V = subss(V,x,inv(T)*x);
end

%% shift back to x0
V = subss(V,x,x-x0);

end

%% for the bilinear search

function [L1,sigma1] = findL1(x,f,V,Lxmonom,options)
  prog = mssprog;

  % construct multipliers for Vdot
  [prog,l] = new(prog,length(Lxmonom),'free');
  L1 = l'*Lxmonom;

  % construct Vdot
  Vdot = diff(V,x)*f;
  
  % construct slack var
  [prog,sigma1] = new(prog,1,'pos');

  % setup SOS constraints
  prog.sos = -Vdot + L1*(V - 1) - sigma1*V;
  prog.sos = L1;

  % run SeDuMi and check output
  [prog,info] = sedumi(prog,-sigma1,0);
  if (info.numerr>1)
    error('sedumi had numerical issues.');
  end
  if (info.pinf || info.dinf)
    error('problem looks infeasible.');
  end

  L1 = prog(L1);
  sigma1 = prog(sigma1);
end

function L2 = findL2(x,V,V0,rho,Lxmonom,options)
  prog = mssprog;

  % construct multipliers
  [prog,l] = new(prog,length(Lxmonom),'free');
  L2 = l'*Lxmonom;
  
  [prog,slack] = new(prog,1,'pos');
  
  prog.sos = -(V-1) + L2*(V0-rho);
  prog.sos = L2;
  
  [prog,info] = sedumi(prog,slack,0);
  if (info.numerr>1)
    error('sedumi had numerical issues.');
  end
  if (info.pinf || info.dinf)
    error('problem looks infeasible.');
  end  

  L2 = prog(L2);
end

function [V,rho]=optimizeV(x,f,L1,L2,V0,sigma1,Vxmonom,options)
  prog = mssprog;
  
  % construct V
  [prog,l] = new(prog,length(Vxmonom),'free');
  V = l'*Vxmonom;
  Vdot = diff(V,x)*f;
  
  % construct rho
  [prog,rho] = new(prog,1,'pos');
  [prog,sigma2] = new(prog,1,'pos');
  
  % setup SOS constraints
  prog.sos = -Vdot + L1*(V - 1) - sigma1*V/2;
  prog.sos = -(V-1) + L2*(V0 - rho); 
  prog.sos = V-sigma2*x'*x;
  
  % run SeDuMi and check output
  [prog,info] = sedumi(prog,-rho,0);
  if (info.numerr>1)
    error('sedumi had numerical issues.');
  end
  if (info.pinf || info.dinf)
    error('problem looks infeasible.');
  end

  V = prog(V);
  rho = double(prog(rho));
end


%% Pablo's method (jointly convex in rho and lagrange multipliers)
function V = pabloMethod(x,V,Vdot,options)
  prog = mssprog;
  Lmonom = monomials(x,0:options.degL1);

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
  
  rho = doubleSafe(prog(rho));
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
  [L,c]=polynomial(xs,options.degL1);

  cnst = sos( (xs'*xs)*(Vs-rho) + L*Vdots );
  
  diagnostics=solvesos(cnst,-rho,[],[rho;c]);
  if (diagnostics.problem~=0)
    error(yalmiperror(diagnostics.problem));
  end
  rho = double(rho)
  
  V = V/rho;
end

%% Line search

function V = rhoLineSearch(x,V,Vdot,options)
  prog = mssprog;

  Lmonom = monomials(x,0:options.degL1);
  [prog,l] = new(prog,length(Lmonom),'free');
  L1 = l'*Lmonom;
  prog.sos = L1;
    
  %% bracket the solution
  rhomin=0; rhomax=1;
  while ( checkRho(rhomax, x,V,Vdot,prog,L1) > 0 )
    rhomin = rhomax;
    rhomax = 1.2*rhomax;
  end
  
  %% now do binary search (mark's version might be better here)
  rho = fzero(@(rho) checkRho(rho, x,V,Vdot,prog,L1),[rhomin rhomax])

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

function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

