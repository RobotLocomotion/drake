function bound = stochasticFiniteTimeVerification(sys,S,ts,x0,options)
% Implements the stochastic verification from Steinhardt11 (Jacob's RSS paper)
% 
% @param S a (potentially time-varying) quadratic form that defines the region of interest ((x-x0)'S(x-x0)<1)
% @param ts time span [t0 tf].  
% @param x0 the (potentially time-varying) nominal state
%
% Given the region of interest (x-x0)'*S*(x-x0) < 1, this implementation of the 
% algorithm searches for a c-super-martingale of the form
%   B = exp((x-x0)'*(S*rho)*(x-x0)) - 1
% which (approximately) tries to optimize the stochastic bound, by performing
% an alternating search to maximize rho and minimize c.  

if (~isCT(sys)) error('only handle CT case so far'); end
if (~isTI(sys)) error('only for TI systems so far'); end

num_x = sys.getNumStates;

%% handle inputs
if (nargin<2 || isempty(S)) S=eye(sys.getNumStates()); end
typecheck(S,'double');
sizecheck(S,[num_x,num_x]);

typecheck(ts,'double');
sizecheck(ts,2);
T = ts(end)-ts(1);

typecheck(x0,'double');  % todo: support trajectories here
sizecheck(x0,[num_x,1]);

if (nargin<6 || isempty(options)) options=struct(); end

if (~isfield(options,'max_iterations')) options.max_iterations=10; end
if (~isfield(options,'converged_tol')) options.converged_tol=.0001; end
if (~isfield(options,'degL')) options.degL = 2; end
if (~isfield(options,'rho0')) options.rho0 = 1; end

%% preprocess dynamics  % todo: handle polynomial trajectories
dyn = sys.p_stochastic_dynamics;
if (getNumInputs(sys)>0)
  dyn = subs(dyn,sys.p_u,0*sys.p_u);
end
f = subs(dyn,sys.p_w,0*sys.p_w);
g = diff(dyn,sys.p_w);
g0 = doubleSafe(subs(g,[sys.p_t;sys.p_x],[ts(1);x0]));

rho = options.rho0;

%% perform bilinear alternations
fprintf('|  rho\t|   c\t| prob\t|\n');
for iter=1:options.max_iterations
  last_rho = rho; 
  [c,L]=findMultipliers(sys.p_t,sys.p_x,f,g,S,rho,options);
  c=1.1*c;  % give a little wiggle room for rho to grow
  rho=optimizeRho(sys.p_t,sys.p_x,f,g,S,c,L,options);
  
  fprintf('| %3.3f\t| %3.3f\t| %3.3f\t|\n',[rho,c,(martingale(S*rho,x0) + c*T)/(exp(rho)-1)]);

  % check for convergence
  if (abs(rho - last_rho) < options.converged_tol*last_rho)  % see if it's converged
    % compute the actual c again.
    [c,L]=findMultipliers(sys.p_t,sys.p_x,f,g,S,rho,options);
    break;
  end
end

%% plot some things (useful for debugging)
% xs = linspace(-2,2,21);
% for i=1:length(xs)
%   B(i) = martingale(S*rho,xs(i));
%   [AB(i),AB1(i),AB2(i)] = expectedDeriv(sys,f,g,S*rho,0,0,xs(i));
% end
% subplot(2,1,1);plot(xs,B); xlabel('x');ylabel('B');
% subplot(2,1,2);plot(xs,AB,xs,AB1,xs,AB2); xlabel('x');legend('AB','AB1','AB2'); 

%% compute actual bound
boundFromOrigin = (martingale(S*rho,x0) + c*T)/(exp(rho)-1)
% sanity check: this one only works for scalar x
%boundFromOrigin = (martingale(S*rho,x0) + c*T)/martingale(S*rho,sqrt(1/S))


end

function B = martingale(S,x)  % note: jacob's draft in elib uses exp(.5*x'*S*x)-1
  B = exp(x'*S*x)-1;
end

function [AB,AB1,AB2]=expectedDeriv(sys,f,g,S,Sdot,t,x)
  f = doubleSafe(subs(f,[sys.p_t;sys.p_x],[t;x]));
  g = doubleSafe(subs(g,[sys.p_t;sys.p_x],[t;x]));

  AB = exp(x'*S*x) * (x'*Sdot*x + 2*x'*S*f + trace(g'*S*g)+2*x'*S*g*g'*S*x);

  % just useful for debugging:
  AB1 = exp(x'*S*x) * (x'*Sdot*x + 2*x'*S*f); % the deterministic (Lyapunov) part
  AB2 = exp(x'*S*x) * trace(g'*(S+2*S*x*x'*S)*g); % the stochastic parts
end

function [c,L] = findMultipliers(t,x,f,g,S,rho,options)
  prog = mssprog;
  [prog,c] = new(prog,1,'pos');
  Lxmonom=monomials(x,0:options.degL);
  [prog,lcoef] = new(prog,length(Lxmonom),'free');
  L = lcoef'*Lxmonom;
  Sr=S*rho;
  
  f=subs(f,t,0*t); g=subs(g,t,0*g); % todo: handle time. 
  prog.sos = c*(1-x'*Sr*x)-2*x'*Sr*f-trace(g'*Sr*g)-2*x'*Sr*g*g'*Sr*x+L*(x'*S*x-1);
  prog.sos = L;
  
  [prog,info] = sedumi(prog,c,0); %1,struct());
  if (info.numerr>1)
    error('sedumi failed due to numerical errors.  try decreasing options.degL.'); 
  end
  if (info.pinf || info.dinf)
    error('problem looks infeasible.  try decreasing options.rho0.');
  end
  c = doubleSafe(prog(c));  
  L = prog(L);
end

function rho=optimizeRho(t,x,f,g,S,c,L,options)
  prog = mssprog;
  [prog,rho] = new(prog,1,'pos');
  Sr=S*rho;
  
  f=subs(f,t,0*t); g=subs(g,t,0*g); % todo: handle time. 

%  prog.sos = c*(1-x'*Sr*x)-2*x'*Sr*f-trace(g'*Sr*g)-2*x'*Sr*g*g'*Sr*x+L*(x'*S*x-1);
%  oops:  this is not linear in rho, so here's the schur complement 
  prog.sss = [ .5, g'*Sr*x; x'*Sr*g, c*(1-x'*Sr*x)-2*x'*Sr*f-trace(g'*Sr*g) + L*(x'*S*x-1) ]; 

  [prog,info] = sedumi(prog,-rho,0);
  if (info.numerr>1)
    error('sedumi failed due to numerical errors');
  end
  if (info.pinf || info.dinf)
    error('problem looks infeasible.');
  end
  rho = doubleSafe(prog(rho));
end

function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end
