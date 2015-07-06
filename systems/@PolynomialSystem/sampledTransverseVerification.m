function [VtrajXP,VtrajX] = sampledTransverseVerification(sys,G,Vtraj0,ts,xtraj,utraj,transSurf,options)
% Attempts to find the largest funnel around xtraj using transverse
% surfaces.
% 
% Implements the algorithm described in "Transverse Dynamics and Regions of Stability for
% Nonlinear Hybrid Limit Cycles" by Ian R. Manchester
%   http://arxiv.org/abs/1010.2241
%
% @param sys plant
% @param G goal region (in one less dimension then the dynamics
% @param Vtraj0 initial guess at the lyapunov function
% @param ts time span to verify over
% @param xtraj trajectory to verify over
% @param utraj control trajectory
% @param transSurf TransversalSurface to use, usally created with
%    TransversalSurface.design()
% @param options structure
%  
% @option rho0_tau initial guess at scaling
%   @default 2
% @option max_iterations maximum number of times to do bilinear
%    optimization
%   @default 10
% @option convered_tol convergence tolerance
%   @default 0.01
% @option stability unimplemented (leave as false)
% @option plot_rho plot rho during optimization
%   @default true
% @option degL1

ts=ts(:);

if (~isCT(sys)) error('not implemented yet'); end

checkDependency('sedumi');
ok=checkDependency('distcomp');  % initialize toolbox if it exists

num_x = sys.getNumStates();
num_xd = sys.getNumDiscStates();
num_xc = sys.getNumContStates();

%%% todo:  really need to make this better (e.g., by updating transSurf in
%%% transverseLQRClosedLoop).  for now, just ignore the variable from tau
num_xc = size(xtraj,1);
% end todo

if (num_xd) error('not implemented yet'); end
num_u = sys.getNumInputs();

num_xp = num_xc-1; 

% make polys
p_x=sys.p_x;
p_t=msspoly('t',1);
p_xp = msspoly('p',num_xp);
%p_tau = msspoly('s',1);

if (nargin<8) options = struct(); end
if (~isfield(options,'rho0_tau')) options.rho0_tau=2; end
if (~isfield(options,'max_iterations')) options.max_iterations=10; end
if (~isfield(options,'converged_tol')) options.converged_tol=.01; end
if (~isfield(options,'stability')) options.stability=false; end  % true implies that we want exponential stability
if (~isfield(options,'plot_rho')) options.plot_rho = true; end

if (options.stability) error('not implemented yet'); end
if (isfield(options,'monom_order')) error('monom_order is no longer used.  try degL1'); end

if (isnumeric(G) && ismatrix(G) && all(size(G)==[num_xp,num_xp]))
  xf=xtraj.eval(ts(end));
  xpf = transSurf.getPi(ts(end))*xf;
  G = (p_xp-xpf)'*G*(p_xp-xpf);
end
typecheck(G,'msspoly');
sizecheck(G,[1 1]);

typecheck(Vtraj0,'PolynomialTrajectory');
sizecheck(Vtraj0,[1 1]);


%% for now, let's require that G matches V at the final conditions
if (~equalpoly(G,Vtraj0.eval(ts(end))))
  error('for now, I require that G matches V at the final conditions');
end  

rhof = .01;  % todo: handle the more general case and get it from containment

N = length(ts);
Vmin = zeros(N-1,1);

%xdottraj = fnder(xtraj);

% evaluate dynamics and Vtraj at every ts once (for efficiency/clarity)
for i=1:N
  tau=ts(i);
  xstar=xtraj.eval(tau);
  z = transSurf.z.eval(tau);
  zdot = transSurf.zdot.eval(tau);
  [Pi Pidot] = transSurf.getPi(tau);

  f=sys.p_dynamics_traj.eval(tau);
  % hack to remove state from controller:
  f=f(1:num_xc); %drop last state dynamics
  f=subss(f,p_x(num_xc+1),tau);
  % end hack

  fp=subss(f,p_x(1:num_xc),xstar+Pi'*p_xp);  
  if (num_u)
    u0=utraj.eval(ts(i));
    fp=subs(fp,sys.p_u,u0);
  end
%  fstar = xdottraj.eval(tau);
  fstar = subs(fp,p_xp,0*p_xp);

  taudot_n = z'*fp;
  taudot_d = z'*fstar - zdot'*Pi'*p_xp;
  
  V=subss(Vtraj0.eval(ts(i)),p_xp,p_xp+Pi*xstar);  % move everything to the origin
  dVdtau = subss(Vtraj0.deriv(tau),p_xp,p_xp+Pi*xstar); 
  dVdxp = diff(V,p_xp);
  
  % compute this now for balancing, but will need to compute it again later
  % (because it depends on rho)
  DV = dVdtau*taudot_n + dVdxp*(taudot_n*Pidot*Pi'*p_xp + ...
    taudot_d*Pi*fp - Pi*fstar*taudot_n);
  
  % balancing 
  S1=.5*doubleSafe(subs(diff(diff(V,p_xp)',p_xp),p_xp,0*p_xp));
  S2=.5*doubleSafe(subs(diff(diff(DV,p_xp)',p_xp),p_xp,0*p_xp));
  [T,D] = balanceQuadForm(S1,S2);

  % apply balancing, and save it all for use downstream
  p_xp_bal = T*p_xp;
%  p_xp_bal = p_xp;  % use this to test without balancing.
  precomp{i}.p_xp=p_xp_bal;
  precomp{i}.tau=tau;
  precomp{i}.Pi=Pi;
  precomp{i}.Pidot=Pidot;
  precomp{i}.fstar=fstar;
  precomp{i}.V=subss(V,p_xp,p_xp_bal);
  precomp{i}.dVdtau=subss(dVdtau,p_xp,p_xp_bal);
  precomp{i}.dVdxp=subss(dVdxp,p_xp,p_xp_bal);
  precomp{i}.taudot_n=subss(taudot_n,p_xp,p_xp_bal);
  precomp{i}.taudot_d=subss(taudot_d,p_xp,p_xp_bal);
  precomp{i}.fp=subss(fp,p_xp,p_xp_bal);
  
end

if (~isfield(options,'degL1'))
  options.degL1 = deg(DV,p_xp) - deg(V,p_xp);  % just a guess
end

% conservative initial guess (need to do something smarter here)
dts = diff(ts);
rho = flipud(rhof*exp(-options.rho0_tau*(ts-ts(1))/(ts(end)-ts(1))))+max(Vmin);
rhodot = diff(rho)./dts;

% check accuracy by sampling
for i=N-1:-1:1
  m(i)=sampleCheck(precomp{i},rho(i),rhodot(i));
end
if (max(m)>0)
  figure(4);clf;fnplt(foh(ts,rho')); axis tight;
  figure(5);clf;plot(ts(1:end-1),m); axis tight;
  drawnow;
  error('infeasible rho. increase options.rho0tau');
end

% perform bilinear search the actual verification here
rhointegral=0;
for iter=1:options.max_iterations
  last_rhointegral = rhointegral; 
  L=findMultipliers(p_xp,precomp,rho,rhodot,options);
  [rho,rhointegral]=optimizeRho(p_xp,precomp,L,dts,rhof,options);
  rhodot = diff(rho)./dts;
  rhopp=foh(ts,rho');

  % plot current rho
  if (options.plot_rho)
    figure(10); fnplt(rhopp); title(['iteration ',num2str(iter)]); drawnow;
  end
  
  % check for convergence
  if ((rhointegral - last_rhointegral) < options.converged_tol*last_rhointegral)  % see if it's converged
    break;
  end
end


VtrajXP = PolynomialTrajectory(@(t) Vtraj0.eval(t)/ppvalSafe(rhopp,t),unique([Vtraj0.getBreaks(),ts']));

if (nargout>1)
  % return the scaled V, switched to x coordinates (instead of xp)
  VtrajX = PolynomialTrajectory(@(t) subss(Vtraj0.eval(t),p_xp,transSurf.getPi(t)*p_x(1:num_xc))/ppvalSafe(rhopp,t),unique([Vtraj0.getBreaks(),ts']));
end

end

function [DV,taudot_d]=transverseConditions(precomp,rho,rhodot)
  % decompress here (for clarity below!)
  p_xp = precomp.p_xp;
  tau = precomp.tau;
  Pi = precomp.Pi;
  Pidot = precomp.Pidot;
  fstar = precomp.fstar;
  V = precomp.V;
  dVdtau = precomp.dVdtau;
  dVdxp = precomp.dVdxp;
  taudot_n = precomp.taudot_n;
  taudot_d = precomp.taudot_d;
  fp = precomp.fp;
  
  % incorporate rho
  dVdtau = dVdtau*rho - V*rhodot;  % intentionally skip 1/rho^2 term (to be multiplied in everywhere else)
  dVdxp = rho*dVdxp;  % 1/rho -> rho because it's multiplied by the rho^2
  
  % compute eq 27
  DV = dVdtau*taudot_n + dVdxp*(taudot_n*Pidot*Pi'*p_xp + ...
    taudot_d*Pi*fp - Pi*fstar*taudot_n);
  
  % current best guess:  Pi*(fp-fstar) looks like it's the wrong sign.
end


% fix lagrange multipliers, optimize rho
function [rho,rhointegral]=optimizeRho(xp,precomp,L,dts,rhof,options)
  N = length(precomp)-1;
  prog = mssprog;
  [prog,rho] = new(prog,N,'pos');
  rho = [rho;rhof];
  
  rhointegral=0;
  for i=1:N
    rhodot(i,1) = (rho(i+1)-rho(i))/dts(i);
    rhointegral = rhointegral+rho(i)*dts(i)+.5*rhodot(i)*dts(i)^2;

    V=precomp{i}.V;
    [DV,taudot_d] = transverseConditions(precomp{i},rho(i),rhodot(i));
    
    prog.sos = -DV-L{i}{1}*(rho(i)-V);
    prog.sos = taudot_d - L{i}{2}*(rho(i)-V);
  end
%  prog = sedumi(prog,ones(1,N)*slack-rhointegral,1,struct());
  [prog,info] = sedumi(prog,-rhointegral,0); %1,struct());
  if (info.numerr>1)
    keyboard;
    error('sedumi failed due to numerical errors'); 
  end
  rho = doubleSafe(prog(rho));
  rhointegral = doubleSafe(prog(rhointegral));
end


% fix rho, optimize lagrange multipliers
function L=findMultipliers(xp,precomp,rho,rhodot,options)
  % note: compute L for each sample point in parallel using parfor

  N = length(precomp)-1;
 
  parfor i=1:N
%   for i=fliplr(1:N)
    prog = mssprog;
    Lxmonom = monomials(xp,0:options.degL1);
    [prog,l] = new(prog,length(Lxmonom),'free');
    L1 = l'*Lxmonom;
    [prog,l] = new(prog,length(Lxmonom),'free');
    L2 = l'*Lxmonom;
    
    V = precomp{i}.V;
    [DV,taudot_d] = transverseConditions(precomp{i},rho(i),rhodot(i));
        
    [prog,gamma] = new(prog,2,'pos');
    prog.sos = -gamma(1)-DV-L1*(rho(i)-V);
    prog.sos = -gamma(2)+taudot_d - L2*(rho(i)-V);
    
    [prog,info{i}] = sedumi(prog,-sum(gamma),0);
    if (info{i}.pinf==0 && info{i}.dinf==0)
      slack{i}=double(prog(gamma));
      L{i}{1} = prog(L1);
      L{i}{2} = prog(L2);
    end
  end
  
  for i=fliplr(1:N)
    if (info{i}.pinf~=0 || info{i}.dinf~=0)
      info{i}
      slack{i}
      error('rho is infeasible');
    end
  end
end


function m=sampleCheck(precomp,rho,rhodot)
  V=precomp.V;
  x=decomp(V);
  
  if (deg(V,x)>2) error('only checks quadratics'); end

  DV = transverseConditions(precomp,rho,rhodot);
  
  
  n=length(x);
  K=100;
  X = randn(n,K);
  X = X./repmat(sqrt(sum(X.^2,1)),n,1);

  H = doubleSafe(0.5*diff(diff(V,x)',x));
  b = -0.5*(H\doubleSafe(subs(diff(V,x),x,0*x)'));

  try 
  X = repmat(b,1,K) + (H/(doubleSafe(rho-subs(V,x,b))+eps))^(-1/2)*X;
  catch
    keyboard;
  end
  m=max(doubleSafe(msubs(DV,x,X)));
  if (m>0)
    warning('found a positive DV');
  end
end

function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

function tf=equalpoly(A,B)
  tf = (clean(A-B,1e-4)==0);
end
