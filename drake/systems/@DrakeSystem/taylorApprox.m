function polysys=taylorApprox(sys,varargin)
% performs a taylorApproximation around a point or trajectory
% usage:
%    taylorApprox(sys,t0,x0,u0,order[,ignores])
% or taylorApprox(sys,x0traj,u0traj,order[,ignores])
% it returns a polynomial system (or polynomial trajectory system)
%
% if u0 or u0traj is [], then the default input (all zeros in the
% input frame) is used

num_x=sys.getNumStates();
num_xc=sys.getNumContStates();
num_xd=sys.getNumDiscStates();
num_u=sys.getNumInputs();
num_y=sys.getNumOutputs();

if (sys.getNumStateConstraints()>0 || sys.getNumUnilateralConstraints()>0)
  warning('Drake:DrakeSystem:TaylorApprox:UnsupportedStateConstraints','This system has state constraints that are being passed to the taylor approximated version (yet)');  
end

if (num_x), p_x = sys.getStateFrame.getPoly; else p_x=[]; end
if (num_u), p_u = sys.getInputFrame.getPoly; else p_u=[]; end

if (length(varargin)<1), error('usage: taylorApprox(sys,t0,x0,u0,order), or taylorApprox(sys,xtraj,utraj,order)'); end

if (isa(varargin{1},'Trajectory'))
  if (length(varargin)<3), error('trajectory usage: taylorApprox(sys,xtraj,utraj,order)'); end
  x0traj = varargin{1};
  u0traj = varargin{2};
  order = varargin{3};
  if (length(varargin)<4), ignores=[]; else ignores=varargin{4}; end

  p_xu = [p_x; p_u];

  typecheck(x0traj,'Trajectory');
  sizecheck(x0traj,num_x);

  % note: i should probably search for the transform before kicking out an error
  if (x0traj.getOutputFrame ~= sys.getStateFrame), error('x0traj does not match state frame'); end
  if (isempty(u0traj))
    % make an empty trajectory object
    u0traj = ConstantTrajectory(zeros(num_u,1));
    u0traj = setOutputFrame(u0traj,sys.getInputFrame);
    breaks = x0traj.getBreaks();
  else
    typecheck(u0traj,'Trajectory');
    sizecheck(u0traj,num_u);
    if (u0traj.getOutputFrame ~= sys.getInputFrame), error('u0traj does not match input frame'); end
    breaks = unique([x0traj.getBreaks(),u0traj.getBreaks()]);
  end

  if (num_xc)
    xdothat = PolynomialTrajectory(@(t)build_poly(@sys.dynamics,t,x0traj.eval(t),u0traj.eval(t),order,p_xu,ignores),breaks);
  else
    xdothat=[];
  end

  if (num_xd)
    xnhat = PolynomialTrajectory(@(t)build_poly(@sys.update,t,x0traj.eval(t),u0traj.eval(t),order,p_xu,ignores),breaks);
  else
    xnhat=[];
  end

  if (num_y)
    yhat = PolynomialTrajectory(@(t)build_poly(@sys.output,t,x0traj.eval(t),u0traj.eval(t),order,p_xu,ignores),breaks);
  else
    yhat=[];
  end

  polysys = PolynomialTrajectorySystem(sys.getInputFrame,sys.getStateFrame,sys.getOutputFrame,xdothat,xnhat,yhat,sys.isDirectFeedthrough());
  polysys = setInputLimits(polysys,sys.umin,sys.umax);

%  if (num_xc==2 && num_xd==0)  % very useful for debugging.  consider making it an option
%    comparePhasePlots(sys,polysys,x0traj,u0traj,linspace(breaks(1),breaks(end),15));
%  end


else
  if (length(varargin)<4), error('this usage: taylorApprox(sys,t0,x0,u0,order)'); end
  if (length(varargin)>4), error('ignores not implemented yet'); end
  t0=varargin{1};
  x0=varargin{2};
  u0=varargin{3};
  order=varargin{4};

  sizecheck(t0,1);
  if (num_x>0)
    typecheck(x0,'Point');
    x0 = double(x0.inFrame(sys.getStateFrame));
  else
    x0=[];
  end
  if isempty(u0) % empty is ok, use default
    if (num_u), u0 = zeros(num_u,1); else u0=[]; end
  else
    typecheck(u0,'Point');
    u0 = double(u0.inFrame(sys.getInputFrame));
  end
  p_t = msspoly('t',1);
  txubar=[p_t-t0;p_x-x0;p_u-u0];

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

  polysys = SpotPolynomialSystem(sys.getInputFrame,sys.getStateFrame,sys.getOutputFrame,xdothat,[],xnhat,yhat);
  if (num_u)
    polysys = setInputLimits(polysys,sys.umin,sys.umax);
  end
  polysys = setInputFrame(polysys,sys.getInputFrame);
  polysys = setStateFrame(polysys,sys.getStateFrame);
  polysys = setOutputFrame(polysys,sys.getOutputFrame);
  polysys = setSampleTime(polysys,sys.getSampleTime);

%  if (num_xc==2 && num_xd==0)   % useful for debugging... consider making it an option
%    x0traj = FunctionHandleTrajectory(@(t) x0, [num_x,1],[0 0]);
%    xdot0traj = FunctionHandleTrajectory(@(t) zeros(num_x,1),[num_x,1],[0 0]);
%    if (isempty(u0))
%      u0traj = FunctionHandleTrajectory(@(t)zeros(0),[0 0],x0traj.tspan);
%    else
%      u0traj = FunctionHandleTrajectory(@(t) u0, [num_u,1],[0 0]);
%    end
%
%    comparePhasePlots(sys,polysys,x0traj,u0traj,0,xdot0traj);
%  end

end

end

  function p=build_poly(fun,t,x0,u0,order,p_xu,ignores)
    nX=length(x0); nU=length(u0);
    xu0=[x0;u0];
    xu=TaylorVar.init(xu0,order);
    xu(ignores)=xu0(ignores);  % turn leaveout_inds into constants (no grad info)
    x0=xu(1:nX); u0=xu(nX+(1:nU));
    p=getmsspoly(feval(fun,t,x0,u0),p_xu-xu0);
  end


  function comparePhasePlots(sys,psys,x0traj,u0traj,ts) %#ok<DEFNU>

  figure(1);
  xs=x0traj.eval(linspace(ts(1),ts(end),100));

%  if (getNumStates(sys)~=2) error('not implmented yet'); end
  num_xd=getNumDiscStates(sys);
  plotdims = [1 2];
  for t=ts
    clf; hold on;
    x0=x0traj.eval(t); u0=u0traj.eval(t);
    [X1,X2] = ndgrid(linspace(-1,1,11),linspace(-1,1,11));
    X1dot=X1; X2dot=X1;
    pX1dot=X1; pX2dot=X1;
    for i=1:numel(X1)
      x=x0; x(num_xd+plotdims)=x(num_xd+plotdims)+[X1(i);X2(i)];
      xdot = sys.dynamics(t,x,u0);
      X1dot(i)=xdot(plotdims(1)); X2dot(i)=xdot(plotdims(2));
      pxdot = psys.dynamics(t,x,u0);
      pX1dot(i) = pxdot(plotdims(1)); pX2dot(i) = pxdot(plotdims(2));
    end
    quiver(x0(num_xd+plotdims(1))+X1,x0(num_xd+plotdims(2))+X2,X1dot,X2dot,'Color',[0 0 1]);
    quiver(x0(num_xd+plotdims(1))+X1,x0(num_xd+plotdims(2))+X2,pX1dot,pX2dot,'Color',[0 1 0]);
    plot(xs(num_xd+plotdims(1),:),xs(num_xd+plotdims(2),:),'r','LineWidth',3);
    plot(x0(num_xd+plotdims(1)),x0(num_xd+plotdims(2)),'r*','MarkerSize',10);
    axis([x0(num_xd+plotdims(1))-1,x0(num_xd+plotdims(1))+1,x0(num_xd+plotdims(2))-1,x0(num_xd+plotdims(2))+1]);
    title(['t = ',num2str(t)]);
    drawnow;
  end
end
