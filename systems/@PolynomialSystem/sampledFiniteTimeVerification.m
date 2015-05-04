function V = sampledFiniteTimeVerification(sys,ts,G,varargin) 
% Attempts to find the largest funnel, defined by the time-varying
% one-level set of V, which verifies (using SOS over state at finite 
% sample points in time) initial conditions to end inside the one-level set
% of the goal region G at time ts(end).  
%
% V = sampledFinitTimeVerification(sys,ts,G,V0,options)
%   uses the LyapunovFunction V0 as the initial guess
% V = sampledFinitTimeVerification(sys,ts,G,x0,options)
%   attempts to construct a LyapunovFunction around the trajectory x0
% 
% @param sys PolynomialSystem that you want to verify.
% @param ts sample times
% @param G the goal region.  can be a PolynomialLyapunovFunction or a nX by
% nX matrix which defines a quadratic form centered at the goal.
% @param V0 a PolynomialLyapunovFunction which is the initial guess
% @param x0 a Trajectory in the state frame of sys around which an initial
% Lyapunov candidate is defined.
% 
% @option rho0_tau initial rho (set large to make the initial guess drop
%   rho quickly)
% @option max_iterations limit on number of alternations in bilinear search
% @option converged_tol tolerance (in units of cost-to-go) for bilinear
% alternations
% @option stability set to true to additionally verify that trajectories
% converge (exponentially) to x0 (or the origin in the V0 frame)
% @option plot_rho set to true for visual progress/debugging output
% @option degL1 polynomial degree of the first lagrange multiplier
% @option degL2 polynomial degree of the second lagrange multiplier
% @option lyap_parameterization specifies whether you search over the
% rescaling of the Lyapunov function (rho) or the entire Lyapunov Function (rhoS).
%
% @retval V a time-varying PolynomialLyapunovFunction who's one-level set
% defines the verified invariant region.
%
% Implements the algorithm described in http://arxiv.org/pdf/1010.3013v1

checkDependency('sedumi');
ok=checkDependency('distcomp');  % initialize toolbox if it exists

t=msspoly('t',1);
ts=ts(:);

num_x = sys.getNumStates();
num_xd = sys.getNumDiscStates();
num_xc = sys.getNumContStates();
if (num_xd), xd = x(1:num_xd); x = x(num_xd + (1:num_xc)); end
num_u = sys.getNumInputs();

if (nargin>4), options = varargin{2};
else options = struct();  end
if (~isfield(options,'rho0_tau')) options.rho0_tau=2; end
if (~isfield(options,'max_iterations')) options.max_iterations=10; end
if (~isfield(options,'converged_tol')) options.converged_tol=.01; end
if (~isfield(options,'stability')) options.stability=false; end  % true implies that we want exponential stability
if (~isfield(options,'plot_rho')) options.plot_rho = true; end
if (~isfield(options,'lyap_parameterization')) options.lyap_parameterization = 'rho'; end %choose to search over rho or rho & S.
if (isa(varargin{1},'Trajectory'))
  x0 = varargin{1}.inFrame(sys.getStateFrame);
  Q = eye(num_xc);
  V0 = tvlyap(sys,x0,Q,Q);
else
  V0 = varargin{1};
end


if (isnumeric(G) && ismatrix(G) && all(size(G)==[num_xc,num_xc]))
  G = QuadraticLyapunovFunction(V0.getFrame,G);
end
typecheck(G,'PolynomialLyapunovFunction');
typecheck(V0,'PolynomialLyapunovFunction');


%% for now, let's require that G matches V at the final conditions
if (~equalpoly(clean(G.getPoly(ts(end))),clean(V0.getPoly(ts(end)))))
  error('for now, I require that G matches V at the final conditions');
end  

rhof = 1;   % todo: handle the more general case and get it from containment

N = length(ts);
Vmin = zeros(N-1,1);

sys = sys.inStateFrame(V0.getFrame); % convert system to Lyapunov function coordinates
x=V0.getFrame.getPoly;
 
% evaluate dynamics and Vtraj at every ts once (for efficiency/clarity)
for i=1:N
  V{i}=V0.getPoly(ts(i));

  f{i} = sys.getPolyDynamics(ts(i));
  if (sys.getNumInputs>0)   % zero all inputs
    f{i} = subs(f{i},sys.getInputFrame.getPoly,zeros(sys.getNumInputs,1));
  end
  
  dVdt{i}=V0.getPolyTimeDeriv(ts(i));
  Vdot{i}=diff(V{i},x)*f{i} + V0.getPolyTimeDeriv(ts(i));

  % balancing 
  if (strcmp(options.lyap_parameterization,'rho'))
  S1=.5*doubleSafe(subs(diff(diff(V{i},x)',x),x,0*x));
  S2=.5*doubleSafe(subs(diff(diff(Vdot{i},x)',x),x,0*x));
  [T,D] = balanceQuadForm(S1,S2);
  Ts{i}=T;
  V{i}=clean(subss(V{i},x,T*x));
  Vdot{i}=clean(subss(Vdot{i},x,T*x));
  f{i}=clean(subss(f{i},x,T*x));
  else
  end
  Vmin(i) = minimumV(x,V{i});

end

if (~isfield(options,'degL1'))
  options.degL1 = deg(Vdot{1},x);  % just a guess
end

% conservative initial guess (need to do something smarter here)
dts = diff(ts);
rho = flipud(rhof*exp(-options.rho0_tau*(ts-ts(1))/(ts(end)-ts(1))))+max(Vmin); 
rhodot = diff(rho)./dts;

% check accuracy by sampling


for i=1:N-1
  m(i)=sampleCheck(x,V{i},Vdot{i},rho(i),rhodot(i));
end


if (max(m)>0)
  figure(4);clf;fnplt(foh(ts,rho')); 
  figure(5);clf;plot(ts(1:end-1),m); drawnow;
  error('Drake:PolynomialTrajectorySystem:InfeasibleRho','infeasible rho. increase options.rho0_tau');
end

V_0=V;
Vdot_0=Vdot;

% perform bilinear search the actual verification here
rhointegral=0;
for iter=1:options.max_iterations
  last_rhointegral = rhointegral; 
  L=findMultipliers(x,V,Vdot,rho,rhodot,options);
  
  if (strcmp(options.lyap_parameterization,'rho'))
      
      [rho,rhointegral]=optimizeRho(x,V,Vdot,L,dts,Vmin,rhof,options);       
      
      rhodot = diff(rho)./dts;
      % plot current rho
      if (options.plot_rho)
        rhopp=foh(ts,rho');
        figure(10); fnplt(rhopp); title(['iteration ',num2str(iter)]); drawnow;
      end

      % check for convergence
      if ((rhointegral - last_rhointegral) < options.converged_tol*last_rhointegral)  % see if it's converged
        break;
      end

      
  elseif (strcmp(options.lyap_parameterization,'rhoS'))
        
        [V, Vdot, Phi,f, rho, rhointegral]=optimize_V2(V_0,Vdot_0,dVdt,f,L,x,rho,rhodot,ts,dts,options);
        rhodot = diff(rho)./dts;
        % plot current rho
        if (options.plot_rho)
        rhopp=foh(ts,rho');
        figure(10); fnplt(rhopp); title(['iteration ',num2str(iter)]); drawnow;
        end

        % check for convergence
        if ((rhointegral - last_rhointegral) < options.converged_tol*last_rhointegral)  % see if it's converged
        break;
        end
    
  else
      
  end 
  
 
end

  if (strcmp(options.lyap_parameterization,'rho'))
      
      V = PPTrajectory(foh(ts,1./rho'))*V0;  % note: the inverse here is an approximation (since 1/rho is not polynomial).  It would be better to rewrite the verification conditions in terms of inv(rho)
    
  elseif (strcmp(options.lyap_parameterization,'rhoS'))
      
     for k=1:length(Phi);Phim(:,:,k)=double(Phi{k});end
     Phipp=PPTrajectory(foh(ts,Phim));     
     V = QuadraticLyapunovFunction(getFrame(V0),V0.S+Phipp,V0.s1,V0.s2);
    V = PPTrajectory(foh(ts,1./rho'))*V;
  end


end



function [V Vdot Phi sigma_integral]=optimize_V1(V0,Vdot0,dVdti,fx,L,x,rho,rhodot,ts,dts,options)

display('Optimizing V...')
tic
N=length(V0);
prog=mssprog;
epsilon=-10^-8;
persistent flag;
persistent Sprev

if isempty(flag)
   
    for k=1:N
       
      S0=   double(diff(diff(V0{k},x)',x)/2); 
      Sprev{k}=S0/rho(k);
        
    end
    
    flag=1;
    
end



xdim=length(x);

sigma_integral=0;

Phi=[];

for k=1:N-1
   [prog phi]=new(prog,xdim,'psd');
   Phi{k}=phi;
end

Phif=double(diff(diff(V0{N},x)',x)/2);

Phi{N}=Phif*0+1;

for k=1:N-1

 
    
    
Phidot=(Phi{k+1}-Phi{k})/dts(k);

S0=   double(diff(diff(V0{k},x)',x)/2);

Sdot0=double(diff(diff(dVdti{k},x)',x)/2);
    
S=S0.*Phi{k};

%S=Phi{k};

Sdot=(Sdot0.*Phi{k})+(S0.*Phidot); 

%Sdot=Phidot;

V{k}=x'*S*x;

dVdt=x'*Sdot*x;

dVdx=diff(V{k},x);

Vdot{k}=dVdx*fx{k}+dVdt;

sigma_integral = sigma_integral+ trace(inv(Sprev{k})*S)/rho(k);%exp(-options.rho0_tau*ts(k)/ts(N))*trace(S);

prog.sos=epsilon-(Vdot{k}-rhodot(k)+L{k}*(V{k}-rho(k)));

   

%prog.sos=epsilon-(Vdot{k}+L{k}*(V{k}-1));

end

S0=double(diff(diff(V0{N},x)',x)/2);

V{N}=x'*S0*x;

[prog,info] = sedumi(prog,sigma_integral,0);

if info.numerr>0
   
    display(['numerical error= ' num2str(info.numerr)]);
    
end

for k=1:N
V{k} = prog(V{k});
end

for k=1:N
       
  S=double(diff(diff(V{k},x)',x)/2); 
  Sprev{k}=S/rho(k);
        
end


for k=1:N-1
Vdot{k}=prog(Vdot{k});
end
% 
% for k=1:N-1
%  %balancing 
%   S1=.5*doubleSafe(subs(diff(diff(V{k},x)',x),x,0*x));
%   S2=.5*doubleSafe(subs(diff(diff(Vdot{k},x)',x),x,0*x));
%   [T,D] = balanceQuadForm(S1,S2);
%   
%   V{k}=subss(V{k},x,T*x);
%   Vdot{k}=subss(Vdot{k},x,T*x);
% end

for k=1:N-1
Phi{k}=double(prog(Phi{k}));
end

sigma_integral=double(prog(sigma_integral));
toc

end


function [V, Vdot, Phi,fx, rho, rhointegral]=optimize_V2(V0,Vdot0,dVdti,fx,L,x,rho0,rhodot0,ts,dts,options)

display('Optimizing V...')
tic
N=length(V0);
prog=mssprog;
epsilon=-10^-8;
persistent flag;
persistent Sprev

if isempty(flag)
   
    for k=1:N
       
      S0=   double(diff(diff(V0{k},x)',x)/2); 
      Sprev{k}=S0;
        
    end
    
    flag=1;
    
end


  [prog,rho] = new(prog,N-1,'pos');
  rho = [rho;1];

xdim=length(x);

sigma_integral=0;

Phi=[];

for k=1:N-1
    [prog,phi0] = new(prog,xdim*xdim,'free');

      Phi{k}=reshape(phi0,xdim,xdim);
end

Phif=double(diff(diff(V0{N},x)',x)/2);

Phi{N}=Phif*0;
rhointegral=0;
for k=1:N-1
    
       rhodot(k,1) = (rho(k+1)-rho(k))/dts(k);
    rhointegral = rhointegral+rho(k)*dts(k);

Phidot=(Phi{k+1}-Phi{k})/dts(k);

S0=   double(diff(diff(V0{k},x)',x)/2);

Sdot0=double(diff(diff(dVdti{k},x)',x)/2);
    
S=S0+Phi{k};

Sdot=Sdot0+Phidot; 

V{k}=x'*S*x;

dVdt=x'*Sdot*x;

dVdx=diff(V{k},x);

Vdot{k}=dVdx*fx{k}+dVdt;

sigma_integral = sigma_integral+ trace(S);

prog.sos=epsilon-(Vdot{k}-rhodot(k)+L{k}*(V{k}-rho(k)));

prog.sos=V{k};

prog.eq=trace(S)-trace(S0);
end

S0=double(diff(diff(V0{N},x)',x)/2);

V{N}=x'*S0*x;

pars.fid = 1;
[prog,info] = sedumi(prog,-rhointegral,0,pars,1);

if info.numerr>0
   
    display(['numerical error= ' num2str(info.numerr)]);
    
end

for k=1:N
V{k} = prog(V{k});
end

rho=double(prog(rho));

for k=1:N
       
  S=double(diff(diff(V{k},x)',x)/2); 
  Sprev{k}=S/rho(k);
        
end


for k=1:N-1
Vdot{k}=prog(Vdot{k});
end

% for k=1:N-1
%  %balancing 
%   S1=.5*doubleSafe(subs(diff(diff(V{k},x)',x),x,0*x));
%   S2=.5*doubleSafe(subs(diff(diff(Vdot{k},x)',x),x,0*x));
%   [T,D] = balanceQuadForm(S1,S2);
%   
%   V{k}=clean(subss(V{k},x,T*x));
%   Vdot{k}=clean(subss(Vdot{k},x,T*x));
%   fx{k}=clean(subss(fx{k},x,T*x));
% end

for k=1:N-1
Phi{k}=double(prog(Phi{k}));
end

sigma_integral=double(prog(sigma_integral));

rhointegral=double(prog(rhointegral));
toc

end

% fix lagrange multipliers, optimize rho
function [rho,rhointegral]=optimizeRho(x,V,Vdot,L,dts,Vmin,rhof,options)
  N = length(V)-1;
  prog = mssprog;
  [prog,rho] = new(prog,N,'pos');
  rho = [rho;rhof]+Vmin;
  
  rhointegral=0;
  for i=1:N
    rhodot(i,1) = (rho(i+1)-rho(i))/dts(i);
    rhointegral = rhointegral+rho(i)*dts(i)+.5*rhodot(i)*dts(i)^2;

    if (options.stability)
      Vdot{i} = Vdot{i}*rho(i)-V{i}*rhodot(i);
      prog.sos = -Vdot{i} + L{i}*(V{i}-rho(i)); 
    else
      prog.sos = -(Vdot{i}-rhodot(i)+L{i}*(V{i}-rho(i)));
    end
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
function L=findMultipliers(x,V,Vdot,rho,rhodot,options)
  % note: compute L for each sample point 

  N = length(V)-1;
 
  parfor i=1:N  
    prog = mssprog;
    Lxmonom = monomials(x,0:options.degL1);
    [prog,l] = new(prog,length(Lxmonom),'free');
    L1 = l'*Lxmonom;
   
    [prog,gamma] = new(prog,1,'free');
    if (options.stability)

%      [prog,l2] = new(prog,length(Lxmonom),'free');
%      L2 = l2'*Lxmonom;
      
      Vdot{i} = Vdot{i}*rho(i)-V{i}*rhodot(i);  % intentionally skip 1/rho^2 term
      prog.sos = gamma*V{i} - Vdot{i} + L1*(V{i}-rho(i));% + L2*(.1-V{i});
      prog.sos = L1;
%      prog.sos = L2;
    else
      prog.sos = gamma-(Vdot{i}-rhodot(i) + L1*(V{i}-rho(i)));
    end
    
    [prog,info{i}] = sedumi(prog,gamma,0);
    
    if (info{i}.pinf==0 && info{i}.dinf==0)
      slack{i}=double(prog(gamma));
      L{i} = prog(L1);
    end
  end
  %slack
 
  for i=fliplr(1:N)
    if (slack{i}>1e-4 || info{i}.pinf~=0 || info{i}.dinf~=0)
      if (length(x)~=2)
        dims=[2;4]; d=ones(length(x),1); d(dims)=0; d=logical(d);
        [m,b]=minimumV(x,V{i});
        V{i}=subs(V{i},x(d),b(d));
        Vdot{i}=subs(Vdot{i},x(d),b(d));
        x=x(dims);
      end
        figure(2); clf; [minVmRho,junk]=plotPoly(x,V{i}-rho(i));
      if (options.stability)
        figure(1); clf; [junk,maxVdot]=plotPoly(x,Vdot{i});
        fprintf(1,'segment %d of %d, slack=%f,\n sampledmax(d/dt(V/rho))=%f, sampledmin(V-rho)=%f\n info:\n',i,N,maxVdot,minVmRho,slack{i});
      else
        figure(1); clf; [junk,maxVdotmRhodot]=plotPoly(x,Vdot{i}-rhodot(i));
        fprintf(1,'segment %d of %d, slack=%f,\n sampledmax(Vdot-rhodot)=%f, sampledmin(V-rho)=%f\n info:\n',i,N,maxVdotmRhodot,minVmRho,slack{i});
      end
      info{i}
      error('rho is infeasible');
    end
  end
end


function [mi,ma]=plotPoly(x,P,rho)
  if(nargin<3) rho=0; end
  [X1,X2]=ndgrid(-2:.1:2,-2:.1:2);
  Ps=reshape(doubleSafe(msubs(P,x,[X1(:)';X2(:)'])),size(X1));
  mi=min(min(Ps));
  ma=max(max(Ps));
  surf(X1,X2,Ps); colorbar;
  view(0,90);
  hold on;
  [c,h]=contour3(X1,X2,Ps,[rho,rho]);
  set(h,'EdgeColor',[1 1 1],'LineWidth',4);
end

function [Vmin,b] = minimumV(x,V)
  if (deg(V,x)>2) 
    prog = mssprog;
    [prog,slack] = new(prog,1,'free');
    prog.sos = slack + V;
    [prog,info] = sedumi(prog,slack,0);
    Vmin = -doubleSafe(prog(slack));
  else
    H = doubleSafe(0.5*diff(diff(V,x)',x));
    b = -0.5*(H\doubleSafe(subs(diff(V,x),x,0*x)'));
    Vmin = subs(V,x,b);
  end    
end

function m=sampleCheck(x,V,Vdot,rho,rhodot)
  if (deg(V,x)>2) error('only checks quadratics'); end
  
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
  m=max(doubleSafe(msubs(Vdot,x,X))) - rhodot;
  if (m>0)
    warning('found a positive Vdot');
  end
end


function tf=equalpoly(A,B)

x=decomp(A);
sizecheck(A,1); sizecheck(B,1);
if (deg(A,x)>2 || deg(B,x)>2) error('not supported yet'); end  % but not very hard!

C=A-B;
if (any(abs(doubleSafe(subs(C,x,0*x)))>1e-4))
  tf=false; return;
end

if (any(abs(doubleSafe(subs(diff(C,x),x,0*x)))>1e-4))
  tf=false; return;
end

if any((any(abs(doubleSafe(subs(diff(diff(C,x)',x),x,0*x)))>1e-4)))
  tf=false; return;
end

tf=true;
end


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

  function V=Lyapunov(t,S,Sdot,Phi,Phidot,x0,p_x,p_t,options)
      V = (p_x-x0)'*(S{1}.*ppval(t,Phi)+(Sdot{1}.*ppval(t,Phi)+S{1}.*ppval(t,Phidot))*(p_t-t))*(p_x-x0);
      %V = (p_x-x0)'*(ppval(t,Phi)+(ppval(t,Phidot))*(p_t-t))*(p_x-x0);
  end
