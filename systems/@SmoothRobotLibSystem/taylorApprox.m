function polysys=taylorApprox(sys,varargin)  
% performs a taylorApproximation around a point or trajectory
% usage:
%    taylorApprox(sys,t0,x0,u0,order)
% or taylorApprox(sys,x0traj,u0traj,order)
% it returns a polynomial system (or polynomial trajectory system) with a
% coordinate system centered at x0 (or x0traj).  (e.g., x0 is now the
% origin)

checkDependency('spot_enabled');

num_x=sys.getNumStates();
num_xc=sys.getNumContStates();
num_xd=sys.getNumDiscStates();
num_u=sys.getNumInputs();
num_y=sys.getNumOutputs();

if (num_x) xbar = msspoly('x',num_x); else xbar=[]; end
if (num_u) ubar = msspoly('u',num_u); else ubar=[]; end

if (length(varargin)<1) error('usage: taylorApprox(sys,t0,x0,u0,order), or taylorApprox(sys,xtraj,utraj,order)'); end

if (isa(varargin{1},'Trajectory'))  
  if (length(varargin)<3) error('trajectory usage: taylorApprox(sys,xtraj,utraj,order)'); end
  x0traj = varargin{1};
  u0traj = varargin{2};
  order = varargin{3};

  sizecheck(x0traj,num_x);
  sizecheck(u0traj,num_u);
  
  xubar=[xbar;ubar];

  if (isempty(u0traj))
    % make an empty trajectory object
    u0traj = FunctionHandleTrajectory(@(t)zeros(0),[0 0],x0traj.tspan);
    breaks = x0traj.getBreaks();
  else
    breaks = unique([x0traj.getBreaks(),u0traj.getBreaks()]);
  end
  
  if (num_xc)
    if (num_xd)
      xdot0traj = fnder(x0traj.subTrajectory(num_xd + (1:num_xc)));
    else
      xdot0traj = fnder(x0traj);
    end
    xdothat = PolynomialTrajectory(@(t)build_poly(@sys.dynamics,t,x0traj.eval(t),u0traj.eval(t),order,xubar,xdot0traj.eval(t)),breaks);
  else
    xdothat=[];
  end

  if (num_xd) 
    xnhat = PolynomialTrajectory(@(t)build_poly(@sys.update,t,x0traj.eval(t),u0traj.eval(t),order,xubar),breaks);
  else
    xnhat=[];
  end
  
  if (num_y)
    yhat = PolynomialTrajectory(@(t)build_poly(@sys.output,t,x0traj.eval(t),u0traj.eval(t),order,xubar),breaks);
  else
    yhat=[];
  end

  polysys = PolynomialTrajectorySystem(xdothat,xnhat,yhat,num_u,sys.isDirectFeedthrough());

  if (0) %num_xc==2 && num_xd==0)  % very useful for debugging.  consider making it an option
    comparePhasePlots(sys,polysys,x0traj,u0traj,linspace(breaks(1),breaks(end),408));
  end

  
else
  if (length(varargin)<4) error('this usage: taylorApprox(sys,t0,x0,u0,order)'); end
  t0=varargin{1};
  x0=varargin{2};
  u0=varargin{3};
  order=varargin{4};

  sizecheck(t0,1);
  sizecheck(x0,num_x);
  sizecheck(u0,num_u);

  tbar = msspoly('t',1);
  txubar=[tbar;xbar;ubar];

  % make TaylorVars
  txu=TaylorVar.init([t0;x0;u0],order);
  pt0=txu(1); px0=txu(1+(1:num_x)); pu0=txu(1+num_x+(1:num_u));

  if (num_xc)
    xdothat = getmsspoly(sys.dynamics(pt0,px0,pu0),txubar);
  else
    xdothat=[];
  end
  
  if (num_xd)
    xnhat = getmsspoly(sys.update(pt0,px0,pu0),txubar);
  else
    xnhat=[];
  end
  
  if (num_y)
    yhat = getmsspoly(sys.output(pt0,px0,pu0),txubar);
  else
    yhat=[];
  end
  
  polysys = PolynomialSystem(num_xc,num_xd,num_u,num_y,sys.isDirectFeedthrough(),sys.isTI(),xdothat,xnhat,yhat);

  if (0) % num_xc==2 && num_xd==0)   % useful for debugging... consider making it an option
    x0traj = FunctionHandleTrajectory(@(t) x0, [num_x,1],[0 0]);
    xdot0traj = FunctionHandleTrajectory(@(t) zeros(num_x,1),[num_x,1],[0 0]);
    if (isempty(u0))
      u0traj = FunctionHandleTrajectory(@(t)zeros(0),[0 0],x0traj.tspan);
    else
      u0traj = FunctionHandleTrajectory(@(t) u0, [num_u,1],[0 0]);
    end

    comparePhasePlots(sys,polysys,x0traj,u0traj,0,xdot0traj);
  end

end
  
end

  function p=build_poly(fun,t,x0,u0,order,xubar,f0,leaveout_inds)
    if (nargin<7) f0=feval(fun,t,x0,u0); end
    if (nargin<8) leaveout_inds=[]; end
    nX=length(x0); nU=length(u0);
    xu0=[x0;u0];
    xu=TaylorVar.init(xu0,order);
    xu(leaveout_inds)=0+xu0(leaveout_inds);  % turn leaveout_inds into constants (no grad info)
    x0=xu(1:nX); u0=xu(nX+(1:nU));
    p=getmsspoly(feval(fun,t,x0,u0)-f0,xubar);
  end
  

  function comparePhasePlots(sys,psys,x0traj,u0traj,ts,xdot0traj)

  figure(1);
  xs=x0traj.eval(linspace(ts(1),ts(end),100));
  if (nargin<6)
    xdot0traj = fnder(x0traj);
  end

  for t=ts
    clf; hold on;
    x0=x0traj.eval(t); u0=u0traj.eval(t); xdot0=xdot0traj.eval(t);
    [Q,Qdot] = ndgrid(x0(1)+linspace(-1,1,11),x0(2)+linspace(-1,1,11));
    Qddot=Q;
    pQdot=Q;
    pQddot=Q;
    for i=1:prod(size(Q))
      xdot = sys.dynamics(t,[Q(i);Qdot(i)],u0);
      Qddot(i) = xdot(2);
      pxdot = xdot0+psys.dynamics(t,[Q(i);Qdot(i)]-x0,zeros(size(u0)));
      pQdot(i) = pxdot(1);
      pQddot(i) = pxdot(2);
    end
    quiver(Q,Qdot,Qdot,Qddot,'Color',[0 0 1]);
    quiver(Q,Qdot,pQdot,pQddot,'Color',[0 1 0]);
    plot(xs(1,:),xs(2,:),'r','LineWidth',3);
    plot(x0(1),x0(2),'r*','MarkerSize',10);
    axis([x0(1)-1,x0(1)+1,x0(2)-1,x0(2)+1]);
    title(['t = ',num2str(t)]);
    drawnow;
  end
  end
