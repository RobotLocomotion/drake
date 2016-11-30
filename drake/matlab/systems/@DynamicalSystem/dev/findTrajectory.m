function [utraj,xtraj] = findTrajectory(sys,x0,xf,sampleFun,con,options)

% Tries to find a feasible solution of the system starting from the 
% initial conditions x0 to and ending in xf.
%
% @param sampleFun 
% @param con see the help for trajectoryOptimization
%
% todo: this will be an interface to many potential methods, like
% trajectoryOptimization, and regionOfAttraction.  But for now it's a
% single, simple forward RRT-like algorithm (using trajectory optimization).


forwardTree = TrajectoryLibrary(sys.getStateFrame);

% add initial condition into forward tree
x0 = ConstantTrajectory(x0); 
x0 = setOutputFrame(x0,sys.getStateFrame);
forwardTree = addTrajectory(forwardTree,x0);

if (nargin<6) options = struct(); end
if ~isfield(options,'MajorOptimalityTolerance') options.MajorOptimalityTolerance=1e-6; end
%if ~isfield(options,'MajorFeasibilityTolerance') options.MajorFeasibilityTolerance=1e-6; end
%if ~isfield(options,'MinorFeasibilityTolerance') options.MinorFeasibilityTolerance=1e-6; end
if ~isfield(options,'MajorIterationsLimit') options.MajorIterationsLimit=50; end
if ~isfield(options,'MinorIterationsLimit') options.MinorIterationsLimit=100; end
if ~isfield(options,'plotdims') options.plotdims = [1,2]; end
if ~isfield(options,'warning') options.warning = false; end

nbreaks=5;
con.T.lb = .1;
%        con.T.ub = nbreaks/10;  % effective dt=.1
con.T.ub = .4;
tf0 = .25;

figure(1); clf; hold on;

while 1
  if rand<.9
    xs = sampleFun();
  else
    xs = xf;
  end

  utraj0 = PPTrajectory(foh(linspace(0,tf0,nbreaks),randn(sys.getNumInputs,nbreaks)));
  con.xf.lb=xs;
  con.xf.ub=xs;
  con.u.lb = sys.umin;
  con.u.ub = sys.umax;
  con.x0.ceq=@forwardTree.distanceSq;
%  con.x0.lb = [0;0];
%  con.x0.ub = [0;0];
  
%  options.tol = 0;
%  gradTest(con.x0.ceq,[0;0],options);
%  gradTest(@(t,x,u)trajCost(t,x,u,.01),0,[0;0],utraj0.eval(0),options);
%  gradTest(@trajFinalCost,0,xs,options);

%  options.grad_method={'taylorvar'};
  [utraj,xtraj,info] = trajectoryOptimization(sys,@(t,x,u)trajCost(t,x,u,.01),@trajFinalCost,xs,utraj0,con,options);
  
  if (info~=1) % failed to find a trajectory
    if ~ismember(info,[13,32])
      [str,cat] = snoptInfo(info);
      warning('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);
    end
    continue;
  end
  
  sfigure(1);
  fnplt(xtraj,options.plotdims);
  drawnow;
end

end


function [g,dg] = trajCost(t,x,u,R)
  g = u'*R*u;
  dg = [0,0*x',2*u'*R];
end
function [h,dh] = trajFinalCost(t,x)
  h = t;
  dh = [1,0*x'];
end

function [J,dJ]=plotDircolTraj(t,x,u,plotdims)
  h=plot(x(plotdims(1),:),x(plotdims(2),:),'.-');
  axis(2*[-pi pi -pi pi]);  % pendulum specific
  drawnow;
  delete(h);
  J = [];
  dJ = [];
%  J=0;
%  dJ=[0*t(:);0*x(:);0*u(:)]';
end


