function linearApproxHJB(sys,gxfun,R,xbins)

% Solves the Hamilton-Jacobi-Bellman equation in a least-squares
% sense for continuous systems (continuous in time,state, and control)
% for the special case of:
%   - time-invariant, control affine dynamics
%   - instantaneous cost function that is quadratic in the input
%        g(x,u) = gx(x) + .5*u'*R*u, R>=0
%   - the value function being estimated by a linear function 
%     approximator: J(x) = alpha'*psi(x)
%
% @param gx is a function handle to the state-dependent part of
%           the instantaneous cost g(x,u) = gx(x) + .5*u'*R*u
% @param R is a pos def matrix representing the input-dependent 
%           part of the cost g(x,u) = gx(x) + .5*u'*R*u
% @param xbins defines the barycentric linear function approximator
%        (note: this will be replaced by a more general linearFA input)

% todo: move this into @DynamicalSystem

assert(isCT(sys));
assert(isTI(sys));
% todo: assert that the system is control affine
typecheck(gxfun,'function_handle');

nx = getNumStates(sys);
nu = getNumInputs(sys);

sizecheck(R,[nu,nu]);
assert(isPositiveDefinite(R));
Rinv = inv(R);

assert(iscell(xbins));
assert(numel(xbins)==nx);

% create the sample points:
Xgrid = cell(1,nx);
[Xgrid{:}] = ndgrid(xbins{:});
X = cellfun(@(a)reshape(a,1,[]),Xgrid,'UniformOutput',false);
X = vertcat(X{:});
nalpha = size(X,2);

% add a bunch of random samples
xmin = reshape(cellfun(@(a)a(1),xbins),[],1);
xmax = reshape(cellfun(@(a)a(end),xbins),[],1);
X = [X,diag(xmax-xmin)*rand(nx,1000)+repmat(xmin,1,1000)];

[ind,coef,dcoef] = barycentricInterpolation(xbins,X);
d = size(coef,1);
nsamples = size(X,2);

% write optimal bellman as alpha'*A*alpha + alpha'b + c
A0 = zeros(nsamples,nalpha);
b0 = zeros(nalpha,1);
C = zeros(nalpha,nsamples*nalpha);

waitbar_h = waitbar(0,'Computing one-step dynamics and transition matrix...');
waitbar_lastupdate = 0;

u_for_f2_extractor = eye(nu);
for i=1:nsamples
  x = X(:,i);
  f1 = dynamics(sys,0,x,zeros(nu,1));
  for j=1:nu
    f2(:,j) = dynamics(sys,0,x,u_for_f2_extractor(:,j));
  end
  
  dpsi = zeros(nalpha,nx);  % could definitely make this more efficient!
  dpsi(ind(:,i),:) = dcoef(d*(i-1)+(1:d),:);
  bigterm = dpsi*f2*Rinv*f2'*dpsi';
  
  A0(i,:) = (dpsi*f1)';
  b0(i) = gxfun(x);
  C(:,nalpha*(i-1)+(1:nalpha)) = bigterm;

  if i/nsamples>waitbar_lastupdate+.025 % don't call the gui too much
    waitbar(i/nsamples,waitbar_h);
    waitbar_lastupdate = i/nsamples;
  end

end
close(waitbar_h);

alpha = rand(nalpha,1);
while(1)
  A = A0 - reshape(alpha'*C,nsamples,nalpha);
  b = b0 + .5*reshape(alpha'*C,nsamples,nalpha)*alpha;
  alpha = A\b;
  
  imagesc(xbins{1},xbins{2},reshape(alpha,numel(xbins{1}),numel(xbins{2}))');
  axis xy;
  xlabel('q');
  ylabel('qdot');
  title('J(x)');
  drawnow;
end
