function V = regionOfAttraction(sys,varargin)
% Estimates the region of attraction.
%    
% V = regionOfAttraction(sys,V0,options)
%   uses the LyapunovFunction V0 as the initial guess
% V = regionOfAttraction(sys,x0,options)
%   attempts to construct a LyapunovFunction around the fixed point x0
%
% The resulting region of attraction is defined by the sub-level set V<=1 
% containing the origin in the LyapunovFunction reference frame.
%
% @param V0 A LyapunovFunction object used as the initial candidate.  
% @param x0 A fixed point of the system (if the system has inputs, they are set to
% zero) @default origin of the system's state frame
%
% @retval V The verified Lyapunov candidate (with V<=1 verified)
%
% @option method The method to use.  Must be one of
% 'bilinear','levelset','levelset_yalmip', 'binary', or 'sampling'.  You may also pass
% in a cell array of methods to run them one after another (e.g. use
% options.method={'levelset','bilinear'} to use the levelset method to
% optimize the initial V0 before starting bilinear alternations).
% 
% @option degV The degree of the Lyapunov function to search over.
% @default 2
% @option degL The degree of the Lagrange multiplier used in the S-procedure.  It's
% implementation is method specific.  Set degL to a vector if there are multiple Langrange multiplier degrees to set.  
% @default the degree required to kill terms in the main polynomial.
%
% @option max_iterations The maximum number of iterations in the iterative
% methods (e.g. biinear)
% @option converged_tol The convergence tolerance in the iterative methods
% (e.g. bilinear).  The algorithm terminates if the estimated volume
% changes by less than this percentage.
% @option optimize Set to false to simply check whether V0 is an ROA.
% @default true
%
% @options numSamples Number of samples to use (on every iteration)q if method is set to
% 'sampling'.  @default 10^(dim(x)+2)
% 

if (~isCT(sys)) error('only handle CT case so far'); end

if ~isempty(sys.state_constraints) error('state constraints not implemented yet'); end
if (isRational(sys)) error('rational dynamics not supported yet'); end
if (~isTI(sys)) error('only works for time-invariant systems (so far)'); end

ok_sedumi = checkDependency('sedumi');
ok_mosek = checkDependency('mosek');

if ~ok_sedumi && ~ok_mosek
  error('Drake:MissingDependency:SDPSolver','You need either MOSEK or SeDuMi installed to use this function.');
end   

%% get Lyapunov candidate
num_x = sys.getNumStates();
if nargin<2 || isa(varargin{1},'Point')
  if nargin<2
    x0 = Point(sys.getStateFrame,zeros(num_x,1));
  else
    x0 = varargin{1}; 
  end
  
  % check that x0 is a fixed point
  x = sys.getStateFrame.getPoly;
  f = sys.getPolyDynamics;
  %% zero all inputs
  if (sys.getNumInputs>0)
    f = subs(f,sys.getInputFrame.getPoly,zeros(sys.getNumInputs,1));
  end
  
  if any(abs(double(subs(f,x,double(x0.inFrame(sys.getStateFrame)))))>1e-5)
    error('x0 must be a fixed point of the system');
  end
    
  % solve a lyapunov equation on the linearized dynamics to get a
  % Lyapunov candidate
  Q = eye(sys.num_x);  % todo: take this as a parameter?
  V = tilyap(sys,x0,Q);
elseif isa(varargin{1},'PolynomialLyapunovFunction')
  V = varargin{1};
else
  error('the second argument must be either a PolynomialLyapunovFunction or a Point representing x0');
end
typecheck(V,'LyapunovFunction');
if ~isTI(V) error('Lyapunov candidate should be time-invariant for region of attraction analysis'); end

sys = sys.inStateFrame(V.getFrame); % convert system to Lyapunov function coordinates
f = sys.getPolyDynamics;
%% zero all inputs
if (sys.getNumInputs>0)
  f = subs(f,sys.getInputFrame.getPoly,zeros(sys.getNumInputs,1));
end
f = f;

%% handle options
if (nargin>2) options=varargin{2};
else options=struct(); end
if (~isfield(options,'method')) 
  options.method={'levelset'}; 
elseif (~iscell(options.method))
  options.method={options.method};
end
if (~isfield(options,'degV')) options.degV = 4; end
if (~isfield(options,'max_iterations')) options.max_iterations=10; end
if (~isfield(options,'converged_tol')) options.converged_tol=.01; end
if (~isfield(options,'optimize')) options.optimize=true; end

if (~isfield(options,'numSamples')) options.numSamples = 10^(num_x+1); end 

if (~isfield(options,'degL1'))
  options.degL1 = options.degV-1 + deg(f,V.getFrame.getPoly);  % just a guess
end
if (~isfield(options,'degL2'))
  options.degL2 = options.degL1;
end

if (options.optimize == false)
  checkConstantRho(V,f,options);
  return;
end

% Choose sdp solver
if ok_mosek
    options.solver = @spot_mosek;
else
    options.solver = @spot_sedumi;
end

for i=1:length(options.method)
  %% compute level set
  switch (lower(options.method{i}))
    case 'bilinear'
      V = bilinear(V,f,options);
    case 'levelset'
      V = levelSetMethod(V,f,options);
    case 'levelset_yalmip'
      V = levelSetMethodYalmip(V,f,options);
    case 'binary'
      V = rhoLineSearch(V,f,options);
    case 'sampling'
      V = sampling(V,f,options);
    otherwise
      error(['don''t know method: ', options.method]);
  end
end

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
    Vbal=subs(V,x,T*x);
    if (nargout>2)
      fbal=inv(T)*subs(f,x,T*x);
    end
  end
end

%% for the bilinear search
function V = bilinear(V0,f,options)

  x = V0.getFrame.getPoly;
  num_x = length(x);
  V=V0;
  
  [T,V0bal,fbal,S0,A] = balance(x,V0.getPoly,f);
  rho = 1;  

  L1monom = monomials(x,0:options.degL1);
  L2monom = monomials(x,0:options.degL2);
  Vmonom = monomials(x,0:options.degV);
  
  vol=0;
  for iter=1:options.max_iterations
    last_vol = vol;
    
    % balance on every iteration (since V and Vdot are changing):
    [T,Vbal,fbal]=balance(x,V.getPoly,f,S0/rho,A);
    V0bal=subs(V0.getPoly,x,T*x);
    
    [L1,sigma1] = findL1(x,fbal,Vbal,L1monom,options);
    L2 = findL2(x,Vbal,V0bal,rho,L2monom,options);
    [Vbal,rho] = optimizeV(x,fbal,L1,L2,V0bal,sigma1,Vmonom,options);
    vol = rho;
    
    % undo balancing (for the next iteration, or if i'm done)
    V = SpotPolynomialLyapunovFunction(V.getFrame,subs(Vbal,x,inv(T)*x));
    plotFunnel(V); 
    plotFunnel(V0/rho,struct('color',[.9 .3 .2])); drawnow;
    
    % check for convergence
    if ((vol - last_vol) < options.converged_tol*last_vol)
      break;
    end
  end
  
end

function [L1,sigma1] = findL1(x,f,V,Lxmonom,options)
  prog = spotsosprog;
  prog = prog.withIndeterminate(x);

  % construct multipliers for Vdot
  [prog,L1] = prog.newFreePoly(Lxmonom);

  % construct Vdot
  Vdot = clean(diff(V,x)*f);
  
  % construct slack var
  [prog,sigma1] = prog.newPos(1);

  % setup SOS constraints
  prog = prog.withSOS(-Vdot + L1*(V - 1) - sigma1*V);
  prog = prog.withSOS(L1);

  % run SeDuMi/MOSEK and check output
  solver = options.solver;
  options = spot_sdp_default_options();
  sol = prog.minimize(-sigma1,solver,options);
  
   if ~sol.isPrimalFeasible
      error('Problem looks primal infeasible');
  end
  
  if ~sol.isDualFeasible
      error('Problem looks dual infeasible. It is probably unbounded. ');
  end

  L1 = sol.eval(L1);
  sigma1 = sol.eval(sigma1);
end

function L2 = findL2(x,V,V0,rho,Lxmonom,options)
  prog = spotsosprog;
  prog = prog.withIndeterminate(x);

  % construct multipliers
  [prog,L2] = prog.newFreePoly(Lxmonom);
  
  [prog,slack] = prog.newPos(1);
  
  prog = prog.withSOS(-(V-1) + L2*(V0-rho));
  prog = prog.withSOS(L2);
  
  solver = options.solver;
  options = spot_sdp_default_options();
  % options.verbose = 1;
  sol = prog.minimize(slack,solver,options);% keyboard;
  
  if ~sol.isPrimalFeasible
      error('Problem looks primal infeasible');
  end
  
  if ~sol.isDualFeasible
      error('Problem looks dual infeasible. It is probably unbounded. ');
  end

  L2 = sol.eval(L2);
end

function [V,rho]=optimizeV(x,f,L1,L2,V0,sigma1,Vxmonom,options)
  prog = spotsosprog;
  prog = prog.withIndeterminate(x);
  
  % construct V
  [prog,V] = prog.newFreePoly(Vxmonom);
  Vdot = diff(V,x)*f;
  
  % construct rho
  [prog,rho] = prog.newPos(1);
  
  % setup SOS constraints
  prog = prog.withSOS(-Vdot + L1*(V - 1) - sigma1*V/2);
  prog = prog.withSOS(-(V-1) + L2*(V0 - rho)); 
  prog = prog.withSOS(V); 
  
  % run SeDuMi/MOSEK and check output
  solver = options.solver;
  options = spot_sdp_default_options();
  sol = prog.minimize(-rho,solver,options);
  
   if ~sol.isPrimalFeasible
      error('Problem looks primal infeasible');
  end
  
  if ~sol.isDualFeasible
      error('Problem looks dual infeasible. It is probably unbounded. ');
  end

  V = sol.eval(V);
  rho = double(sol.eval(rho));
end


%% Pablo's method (jointly convex in rho and lagrange multipliers)
function V = levelSetMethod(V0,f,options)

  x = V0.getFrame.getPoly;
  [T,V,f] = balance(x,V0.getPoly,f);

  %% compute Vdot
  Vdot = diff(V,x)*f;

  % check Hessian Vdot at origin, to make sure it's negative def.
  H=.5*doubleSafe(subs(diff(diff(Vdot,x)',x),x,0*x));  % extract Hessian
  if (~isPositiveDefinite(-H)) error('Vdot must be negative definite at the origin'); end

  prog = spotsosprog;
  prog = prog.withIndeterminate(x);
  Lmonom = monomials(x,0:options.degL1);
%  Lmonom = hermite_basis(monomials(x,0:options.degL1));

  [prog,rho] = prog.newFree(1);
    
  [prog,L] = prog.newFreePoly(Lmonom);

  prog = prog.withSOS((x'*x)^floor((options.degL1 + deg(Vdot)-deg(V))/2)*(V - rho) +  L*Vdot);
  
  solver = options.solver;
  options = spot_sdp_default_options();
  sol = prog.minimize(-rho,solver,options);

  if ~sol.isPrimalFeasible
      error('Problem looks primal infeasible');
  end
  
  if ~sol.isDualFeasible
      error('Problem looks dual infeasible. It is probably unbounded. ');
  end
  
  rho = doubleSafe(sol.eval(rho));
  if (rho<=0) error('optimization failed'); end

  V = V/rho;
  
  %% undo balancing
  V = subs(V,x,inv(T)*x);
  V = SpotPolynomialLyapunovFunction(V0.getFrame,V);
end

function V=levelSetMethodYalmip(V0,f,options)
  checkDependency('yalmip');

  x = V0.getFrame.getPoly;
  [T,V,f] = balance(x,V0.getPoly,f);

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
  V = subs(V,x,inv(T)*x);
  V = SpotPolynomialLyapunovFunction(V0.getFrame,V);
end

%% Line search

function V = rhoLineSearch(V0,f,options)
  x = V0.getFrame.getPoly;
  [T,V,f] = balance(x,V0.getPoly,f);

  %% compute Vdot
  Vdot = diff(V,x)*f;

  prog = spotsosprog;
  prog = prog.withIndeterminate(x);

  Lmonom = monomials(x,0:options.degL1);
  [prog,L1] = prog.newSOSPoly(Lmonom);
 
    
  %% bracket the solution
  rhomin=0; rhomax=1;
  while ( checkRho(rhomax, x,V,Vdot,prog,L1,options) > 0 )
    rhomin = rhomax;
    rhomax = 1.2*rhomax;
  end
  
  %% now do binary search (mark's version might be better here)
  rho = fzero(@(rho) checkRho(rho, x,V,Vdot,prog,L1,options),[rhomin rhomax])

  V = V/rho;

  %% undo balancing
  V = subs(V,x,inv(T)*x);
  V = SpotPolynomialLyapunovFunction(V0.getFrame,V);
end

function [slack,info] = checkConstantRho(V0,f,options)
  x = V0.getFrame.getPoly;
  [T,V,f] = balance(x,V0.getPoly,f);

  %% compute Vdot
  Vdot = diff(V,x)*f;
  
  %% visualize Vdot
%{
  step = 0.5;

  xmin = 8;
  xstep = .25;
  xmax = 10;
  
  ymin = 0;
  ystep = 0.5;
  ymax = 10;
  
  [X, Y] = meshgrid(xmin:xstep:xmax, ymin:ystep:ymax);

  for i = 1:size(X,2)
    for j = 1:size(Y,1)

      Z(j,i) = double(subs(f, x, [X(j,i); Y(j,i);]));
    end
  end

  surf(X,Y,Z)
  
  title(strcat('t=',num2str(t),', x3 (angle)=',num2str(x3val)));
  xlabel('x0');
  ylabel('x1');
  drawnow;
  %}
  
  %% back to the normal script after visualizing
  prog = spotsosprog;
  prog = prog.withIndeterminate(x);

  %Lmonom = monomials(x,0:options.degL1);
  Lmonom = hermite_basis(monomials(x,0:options.degL1));
  
  [prog,L1] = prog.newSOSPoly(Lmonom);
  

  [slack,info] = checkRho(1, x, V, Vdot, prog, L1, options)
  
end
  

function [slack,info] = checkRho(rho,x,V,Vdot,prog,L,options)
  [prog,slack] = prog.newFree(1);
  
  prog = prog.withSOS(-Vdot + L*(V - rho) - slack*V);
  
  solver = options.solver;
  options = spot_sdp_default_options();
  sol = prog.minimize(-slack,solver,options);
  
   if ~sol.isPrimalFeasible
      error('Problem looks primal infeasible');
  end
  
  if ~sol.isDualFeasible
      error('Problem looks dual infeasible. It is probably unbounded. ');
  end
  
  slack = doubleSafe(sol.eval(slack));
end


function V = sampling(V0,f,options)
%  No SOS here.  Just use lots and lots of samples to try to estimate level
%  set of V candidate.

  x = V0.getFrame.getPoly;
  [T,V,f] = balance(x,V0.getPoly,f);

  if (deg(V,x)>2) error('only checks quadratics'); end


  %% compute Vdot
  Vdot = diff(V,x)*f;

  %% bracket the solution
  rhomin=0; rhomax=1;
  while ( samplingCheckRho(rhomax, x,V,Vdot,options) <= 0 )
    rhomin = rhomax;
    rhomax = 1.2*rhomax;
  end
  if (rhomin==0)  % need to find a non-zero rhomin
    rhomin = rhomax/2;
    while (samplingCheckRho(rhomin, x,V,Vdot,options) > 0)
      rhomax = rhomin;
      rhomin = rhomin/2;
    end
  end
  
  %% now do binary search (mark's version might be better here)
  rho=-1;
  while (rho<0)
    try
      rho = fzero(@(rho) samplingCheckRho(rho, x,V,Vdot,options),[rhomin rhomax]);
    catch ex
      if (strcmp(ex.identifier,'MATLAB:fzero:ValuesAtEndPtsSameSign'))
        rho=-1;  % fzero could fail due to randomness in the sampling evaluation.  keep looping until it succeeds.
      else
        throw(ex)
      end
    end
  end
  
  rho
  V = V/rho;

  %% undo balancing
  V = subs(V,x,inv(T)*x);
  V = SpotPolynomialLyapunovFunction(V0.getFrame,V);
end

function m = samplingCheckRho(rho,x,V,Vdot,options)
  % to be consistent with the code above, return max 
  % value of m such that Vdot(x)<-m*V(x).  
  % (m is like the slack variable)
  % if m < 0, then the rho level set is not an ROA
  if (deg(V,x)>2) error('only checks quadratics'); end
  
  % First find a random sample inside the rho ellipse
  %  xs'*H*xs = rho
  % with xs = (H/rho)^{-.5}*xrand
  % where xrand is a random vector of length between 0 and 1
  
  rho
%  h=plotFunnel(V/rho,zeros(size(x)));
  
  n=length(x);
  K=options.numSamples;
  xs = randn(n,K);
  xs = (xs./repmat(sqrt(sum(xs.^2,1)),n,1)).*repmat(rand(1,K),n,1);

  H = doubleSafe(0.5*diff(diff(V,x)',x));
  xs = (H/rho)^(-1/2)*xs;

%  h=[h;plot(xs(1,:),xs(2,:),'.')];

  ms = double(msubs(Vdot,x,xs));
%  ms = -double(msubs(Vdot,x,xs))./double(msubs(V,x,xs));
  m=max(ms);

%  drawnow;
%  delete(h);
end

function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end
