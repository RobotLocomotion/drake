function [Wopt,xs] = sampledFiniteTimeReachability(poly,ts,xtraj,gTmss,R,options)
% Implements Henrion and Korda's SOS relaxation for computing finite time
% funnels. Given an initial set B(0) and a description of
% the dynamics, this function gives a set B(t) 
% corresponding to each time sample such that any state starting in B(0) is 
% "guaranteed" to be in B(t) at time t.  The dynamics are not explicitly modeled as
% a function of time since this requires having a large total degree (in t and
% x) of the approximating polynomial. Rather, the dynamics are sampled at
% finitely many points in time and the verification is performed at each
% sample point (just like the sampledFiniteTimeVerification.m function). 
% This only affects the constraint that checks the "Vdot" condition. The
% two big advantages of this formulation are (1) a single convex program is
% solved to get B(t) for a given time and (2) the computations for different
% B(t)s are separate and thus can be parallelized.
%
% @param poly PolynomialTrajectorySystem that you want to verify
% @param ts Time samples to verify conditions on
% @param xtraj Nominal trajectory about which funnel is computed
% @param gTmss Description of set of initial conditions B(0). gT must be a
% @param R Radius of ball describing X (the big space in which everyting
% happens). Right now, I only allow this to be a ball, but we can easily extend
% this to ellipsoids too. In principle, you can do any semialgebraic set,
% but the number of multipliers required increases with the number of
% polynomials needed to describe the set. Also, Lebesgue moments have to be
% calculated on this set - this can be nontrivial.
% @option clean_tol Tolerance for cleaning stuff in yalmip
% spot polynomial (of arbitrary degree) which is > 0 on B(0).
% @option degV Degree of "Lyapunov function"
% @option degW Degree of "approximating function"
% @option degq1 q1 multiplier degree (if you don't set this, I'll set it 
% automatically by degree matching (based on degV and degW)
% @option degq01 q01 multiplier degree (if you don't set this, I'll set it 
% automatically by degree matching (based on degV and degW)
% @option degqT qT multiplier degree (if you don't set this, I'll set it 
% automatically by degree matching (based on degV and degW)
% @option degs0 s0 multiplier degree (if you don't set this, I'll set it 
% automatically by degree matching (based on degV and degW)
% @option backoff_percent Percentage to back off on objective when
% resolving sos program (backing off helps with numerics)


% Set defaults
if (~isfield(options,'clean_tol')) options.clean_tol=1e-6; end
if (~isfield(options,'degV')) options.degV = 4; end
if (~isfield(options,'degW')) options.degW = 4; end
if (~isfield(options,'degq1')) options.degq1 = options.degV; end
if (~isfield(options,'degq01')) options.degq01 = options.degW - 2; end
if (~isfield(options,'degqT')) options.degqT = options.degV - 2; end
if (~isfield(options,'degs0')) options.degs0 = options.degW - 2; end
if (~isfield(options,'backoff_percent')) options.backoff_percent = 5; end

% Get state
x = poly.getOutputFrame.getPoly;
nX = length(x);

% Get state
x = poly.getOutputFrame.getPoly;

% Time derivative of nominal xtraj
xdottraj = fnder(xtraj);

% Sdpvar for state
xs = sdpvar(length(x),1);

% Compute xdot at each sample point
N = length(ts);
for i = 1:N
    % Get polynomial dynamics at sample point
    fi = poly.getPolyDynamics(ts(i));
    % Zero out input variable
    fi = cleanmsspoly(subs(fi,poly.getInputFrame.getPoly,zeros(poly.getNumInputs,1)),options.clean_tol);
    % Convert to Yalmip
    f = msspoly2sdpvar(x,xs,fi);
    % Do coordinate change to center things about 0
    x0 = xtraj.eval(ts(i));
    f = replace(f,xs,xs + xtraj.eval(ts(i))) - xdottraj.eval(ts(i));

    % Clean expression and reverse time
    fy{i} = clean(-f,options.clean_tol); % Note sign flip - reversing time here
end

% Flip time samples
ts = ts(:); 
dts = flipud(diff(ts));
ts = [0;cumsum(dts)];
fy = fliplr(fy);

% Define spaces X (full space) and xT (target set)
gX = R^2 - sum(xs.^2);
gT = msspoly2sdpvar(x,xs,gTmss);


 

% Compute Lebesgue moments of monomials of W over X
% See "How to Integrate a Polynomial over a Sphere", G. B. Folland for
% details
alphas = monpowers(nX,options.degW);
betas = 0.5*(alphas + 1);
Ra = (R.^(sum(alphas,2) + nX))./(sum(alphas,2) + nX);
IS = 2*prod(gamma(betas),2)./(gamma(sum(betas,2)));
l = Ra.*IS;
alphaszero = (mod(alphas,2) ~= 0);
alphaszero = any(alphaszero,2);
l(alphaszero) = 0;

% Solve the sos program associated with each time sample. All of these
% programs are completely independent and I can solve them in parallel
% but I'm not introducing a parfor yet for debugging purposes.
% if (matlabpool('size')==0) matlabpool('open',4); end

for k = 1:N-1

    tic
    % Initialize Yalmip constraints and coeffs
    C = []; 
    coefs = [];
    
    % Declare V and W 
    vV = monolist(xs,options.degV);
    cV = sdpvar(length(k:N),length(vV)); % One per time step
    [W,cW,vW] = polynomial(xs,options.degW); % Just need one for t = 0
    
    coefs = [coefs;cV(:);cW];
    
    % Multipliers
    vq1 = monolist(xs,options.degq1);
    cq1 = sdpvar(length(k:N),length(vq1));
    [q01,cq01,vq01] = polynomial(xs,options.degq01);
    [qT,cqT,vqT] = polynomial(xs,options.degqT);
    [s0,cs0,vs0] = polynomial(xs,options.degs0);
    
    coefs = [coefs;cq1(:);cq01;cqT;cs0];
    
    % SOS conditions
    
    % First, Vdot conditions at each sample point in time
    for j = k:N-1
        Vj = cV(j-(k-1),:)*vV;
        Vjn = cV(j+1-(k-1),:)*vV;
        Vdotj = jacobian(Vj,xs)*fy{j} + (Vjn - Vj)/(ts(j+1) - ts(j));
        
        C = [C; sos(-Vdotj - cq1(j-(k-1),:)*vq1*gX); sos(cq1(j-(k-1),:)*vq1)];
    end
    
    Con2 = sos(W - cV(1,:)*vV - 1 - q01*gX);
    Con3 = sos(cV(end,:)*vV - qT*gT);
    Con4 = sos(W - s0*gX);
    
    C = [C;Con2;Con3;Con4;sos(q01);sos(qT);sos(s0)];
    
        
    % Solve SOS program
    optionsY = sdpsettings('sedumi.cg.maxiter',100,'verbose',1);
    diagnostics = solvesos(C,cW'*l,optionsY,coefs)
    
    % Back-off and solve again
    obj = double(cW)'*l;
    C = [C; cW'*l < (1 + options.backoff_percent/100)*obj];
    diagnostics = solvesos(C,[],optionsY,coefs)
    
    % Store stuff (with time flipped back properly)
    Wopt{N-k+1} = double(cW)'*vW;
    
    toc
        
end

% Add the initial set 
Wopt{1} = gT + 1;







