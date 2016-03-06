function [K,V] = maxROAFeedback(sys,x0,u0,options)

typecheck(sys,'PolynomialSystem');  % note: this should move to the @PolynomialSystem directory when it moves out of dev

if (nargin<4) options=struct(); end
if (~isfield(options,'degK')) options.degK=1; end
if (~isfield(options,'degV')) options.degV=2; end
if (~isfield(options,'max_iterations')) options.max_iterations=10; end
if (~isfield(options,'converged_tol')) options.converged_tol=.01; end

if (~isCT(sys)) error('only handle CT case so far'); end
if (~isTI(sys)) error('only for TI systems so far'); end

num_x = sys.getNumStates();
num_u = sys.getNumInputs();

if (num_u<1) error('need to have inputs in order to design feedback'); end

if (nargin<2 || isempty(x0))
  x0=zeros(num_x,1);
else
  typecheck(x0,'double');
  sizecheck(x0,[num_x,1]);
end

if (nargin<3 || isempty(u0))
  u0=zeros(num_u,1);
else
  typecheck(u0,'double');
  sizecheck(u0,[num_u,1]);
end

if (options.degV<2) error('V must be deg >=2'); end
if (options.degK<1) error('K must be deg >=1'); end

x = sys.p_x;
u = sys.p_u;

% f is time-invariant zero-input dynamics relative to x0,u0
f = subss(sys.p_dynamics,[sys.p_t;x;u],[0;x + x0;u+u0]);
if (deg(f,u)>1) error('dynamics must be affine in u'); end
%f1 = subs(f,u,0*u); f2 = diff(f,u);

if (~isfield(options,'degL')) options.degL=options.degV-1+options.degK+deg(f,x); end  % degree of Vdot


%% initialize using LQR
A = doubleSafe(subs(diff(f,x),[x;u],zeros(num_x+num_u,1)));
B = doubleSafe(subs(diff(f,u),[x;u],zeros(num_x+num_u,1)));
Q = eye(sys.num_x); R = eye(sys.num_u);  % todo: take these as parameters?
[K0,S0] = lqr(A,B,Q,R);  
K=-K0*x; V=x'*S0*x;  V0 = V; rho=.2; V=V/rho;

%% now run iterations
vol=0;
for iter=1:options.max_iterations
  last_vol = vol;
  
  K = -R^(-1)*diff(f,u)'*diff(V,x)';
  % balance on every iteration (since V and Vdot are changing):
  Vdot = diff(V,x)*subss(f,u,K);
  S1=.5*doubleSafe(subs(diff(diff(V,x)',x),x,0*x));
  S2=.5*doubleSafe(subs(diff(diff(Vdot,x)',x),x,0*x));
  [T,D] = balanceQuadForm(S1,S2);
  Ti = inv(T);
  Vbal=subss(V,x,T*x);  
  V0bal=subss(V0,x,T*x);
  fbal=Ti*subss(f,x,T*x);
  
  [L1,sigma1] = findL1(x,u,fbal,Vbal,R,options);
  L2 = findL2(x,u,Vbal,V0bal,rho,options);
  [Vbal,rho] = optimizeV(x,u,fbal,L1,L2,V0bal,sigma1,R,options);
  vol = rho
  
  % undo balancing (for the next iteration, or if i'm done)
  V = subss(Vbal,x,Ti*x);

  % check for convergence
  if ((vol - last_vol) < options.converged_tol*last_vol)  
    break;
  end
end

%% shift back to x0,u0
K = u0+subss(K,x,x-x0);
V = subss(V,x,x-x0);

end


function [L1,sigma1] = findL1(x,u,f,V,R,options)
  prog = mssprog;

  % construct multipliers for Vdot
  Lxmonom = monomials(x,0:options.degL);
  [prog,l] = new(prog,length(Lxmonom),'free');
  L1 = l'*Lxmonom;

  % construct Vdot
  K = -R^(-1)*diff(f,u)'*diff(V,x)';
  Vdot = diff(V,x)*subss(f,u,K);
  
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

function L2 = findL2(x,u,V,V0,rho,options)
  prog = mssprog;

  % construct multipliers
  Lxmonom = monomials(x,0:options.degL);  % note: make this an independent option (e.g., for quadratics, it should be 0)
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

function [V,rho]=optimizeV(x,u,f,L1,L2,V0,sigma1,R,options)
  prog = mssprog;
  
  % construct V
  Vxmonom = monomials(x,0:options.degV);
  [prog,l] = new(prog,length(Vxmonom),'free');
  V = l'*Vxmonom;

  % construct rho
  [prog,rho] = new(prog,1,'pos');
  [prog,sigma2] = new(prog,1,'pos');
  
  % setup SOS constraints
  f1 = subs(f,u,0*u); f2 = diff(f,u);
  dVdx = diff(V,x);
  
  prog.sss = [-dVdx*f1 + L1*(V - 1) - sigma1*V/2, dVdx*f2; f2'*dVdx', -R];
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

function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end


