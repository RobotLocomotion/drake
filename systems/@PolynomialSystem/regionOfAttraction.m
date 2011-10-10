function V = regionOfAttraction(sys,x0,V0,options)
% Estimates the region of attraction around x0 using V0 as an initial guess.
% The resulting region of attraction is defined by the sub-level set V<=1 surrounding x0.
%
% @param x0 The fixed point (if the system has inputs, they are set to
% zero)
% @param V0 The initial Lyapunov candidate.  
%
% @option method The method to use.  Must be one of
% 'bilinear','levelset','levelset_yalmip', or 'binary'.  You may also pass
% in a cell array of methods to run them one after another (e.g. use
% options.method={'levelset','bilinear'} to use the levelset method to
% optimize the initial V0 before starting bilinear alternations).
% 
% @option degV The degree of the Lyapunov function to search over.
% @default 2
% @option degL1 The degree of the first Lagrange multiplier.  It's
% implementation is method specific.  @default degV + deg(f) - 1 
% @option degL2 The degree of the second Lagrange multiplier, if used.
% @default degL1
%
% @option max_iterations The maximum number of iterations in the iterative
% methods (e.g. biinear)
% @option converged_tol The convergence tolerance in the iterative methods
% (e.g. bilinear).  The algorithm terminates if the estimated volume
% changes by less than this percentage.
% 

if (nargin<4) options=struct(); end
if (~isfield(options,'method')) 
  options.method={'levelset'}; 
elseif (~iscell(options.method))
  options.method={options.method};
end
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

if (~isfield(options,'degL1'))
  options.degL1 = options.degV-1 + deg(f,x);  % just a guess
end
if (~isfield(options,'degL2'))
  options.degL2 = options.degL1;
end

for i=1:length(options.method)
  %% compute level set
  switch (lower(options.method{i}))
    case 'bilinear'
      V = bilinear(x,V,f,options);
    case 'levelset'
      V = levelSetMethod(x,V,f,options);
    case 'levelset_yalmip'
      V = levelSetMethodYalmip(x,V,f,options);
    case 'binary'
      V = rhoLineSearch(x,V,f,options);
    otherwise
      error(['don''t know method: ', options.method]);
  end
end

%% shift back to x0
V = subss(V,x,x-x0);

end


function [T,Vbal,fbal,S,A] = balance(x,V,f,S,A)
  if (nargin<4 || isempty(S))
    S=.5*doubleSafe(subs(diff(diff(V,x)',x),x,0*x));  % extract Hessian
  end
  if (nargin<5 || isempty(A))
    A = doubleSafe(subs(diff(f,x),x,0*x));
  end
  
  [T,D] = balanceQuadForm(S,(S*A+A'*S));
  
  if (nargout>1)
    Vbal=subss(V,x,T*x);
    if (nargout>2)
      fbal=inv(T)*subss(f,x,T*x);
    end
  end
end

%% for the bilinear search
function V = bilinear(x,V0,f,options)

  num_x = length(x);
  V=V0;
  [T,V0bal,fbal,S0,A] = balance(x,V0,f);
  rho = 1;  

  L1monom = monomials(x,0:options.degL1);
  L2monom = monomials(x,0:options.degL2);
  Vmonom = monomials(x,0:options.degV);
  
  vol=0;
  for iter=1:options.max_iterations
    last_vol = vol;
    
    % balance on every iteration (since V and Vdot are changing):
    [T,Vbal,fbal]=balance(x,V,f,S0/rho,A);
    V0bal=subss(V0,x,T*x);
    
    [L1,sigma1] = findL1(x,fbal,Vbal,L1monom,options);
    L2 = findL2(x,Vbal,V0bal,rho,L2monom,options);
    [Vbal,rho] = optimizeV(x,fbal,L1,L2,V0bal,sigma1,Vmonom,options);
    vol = rho
    
    % undo balancing (for the next iteration, or if i'm done)
    V = subss(Vbal,x,inv(T)*x);
    plotFunnel(V,zeros(num_x,1)); plotFunnel(V0/rho,zeros(num_x,1),[],struct('color',[.9 .3 .2])); drawnow;
    
    % check for convergence
    if ((vol - last_vol) < options.converged_tol*last_vol)
      break;
    end
  end
end

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
%  if (info.numerr>1)
%    error('sedumi had numerical issues.');
%  end
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
%  if (info.numerr>1)
%    error('sedumi had numerical issues.');
%  end
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
  
  % setup SOS constraints
  prog.sos = -Vdot + L1*(V - 1) - sigma1*V/2;
  prog.sos = -(V-1) + L2*(V0 - rho); 
  prog.sos = V; 
  
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
function V = levelSetMethod(x,V,f,options)

  [T,V,f] = balance(x,V,f);

  %% compute Vdot
  Vdot = diff(V,x)*f;

  % check Hessian Vdot at origin, to make sure it's negative def.
  H=.5*doubleSafe(subs(diff(diff(Vdot,x)',x),x,0*x));  % extract Hessian
  if (~isPositiveDefinite(-H)) error('Vdot must be negative definite at the origin'); end

  prog = mssprog;
  Lmonom = monomials(x,0:options.degL1);
%  Lmonom = hermite_basis(monomials(x,0:options.degL1));

  rho = msspoly('r');
  prog.free = rho;
    
  [prog,l] = new(prog,length(Lmonom),'free');
  L = l'*Lmonom;

  prog.sos = (x'*x)^floor((options.degL1 + deg(Vdot)-deg(V))/2)*(V - rho) +  L*Vdot;
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
  
  %% undo balancing
  V = subss(V,x,inv(T)*x);
end

function V=levelSetMethodYalmip(x,V,f,options)
  checkDependency('yalmip_enabled');

  [T,V,f] = balance(x,V,f);

  %% compute Vdot
  Vdot = diff(V,x)*f;

  % check Hessian Vdot at origin, to make sure it's negative def. 
  H=.5*doubleSafe(subs(diff(diff(Vdot,x)',x),x,0*x));  % extract Hessian
  if (~isPositiveDefinite(-H)) error('Vdot must be negative definite at the origin'); end

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

  %% undo balancing
  V = subss(V,x,inv(T)*x);
end

%% Line search

function V = rhoLineSearch(x,V,f,options)
  [T,V,f] = balance(x,V,f);

  %% compute Vdot
  Vdot = diff(V,x)*f;

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

  %% undo balancing
  V = subss(V,x,inv(T)*x);
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

