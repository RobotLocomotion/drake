function [utraj,xtraj]=runDircolCycle(p)
% find stable limit cycle 

if (nargin<1)
  p = SpringFlamingoPlant.build();
end
umin = -20; umax=20;

% numbers from the first step of the passive sim 
x0 = getInitialState(p);
m0=x0(1);
x0=x0(2:end);

N=15;
utraj0 = {PPTrajectory(foh(linspace(0,.1,N),zeros(6,N))),...
  PPTrajectory(foh(linspace(0,.1,N),zeros(6,N))),...
  PPTrajectory(foh(linspace(0,.1,N),zeros(6,N)))};%,...
%{
  PPTrajectory(foh(linspace(0,.1,N),zeros(6,N))),...
  PPTrajectory(foh(linspace(0,.25,N),zeros(6,N)))};
%}

% todo: get input and state constraints from 

% single support
con.mode{1}.mode_num=m0;   
con.mode{1}.x0.lb = x0;
con.mode{1}.x0.ub = x0;
con.mode{1}.T.lb = 0;
con.mode{1}.T.ub = .2;
con.mode{1}.u.lb = repmat(umin,6,1);
con.mode{1}.u.ub = repmat(umax,6,1);

% toe-off
con.mode{2}.mode_num = p.modeNumFromGCboolean([0 1 0 0]');
con.mode{2}.x0.lb = [-inf; .8; repmat(-inf,32,1)];
con.mode{2}.x0.ub = [x0(1); repmat(inf,33,1)];
con.mode{2}.T.lb = 0;
con.mode{2}.T.ub = .2;
con.mode{2}.u.lb = repmat(umin,6,1);
con.mode{2}.u.ub = repmat(umax,6,1);

% heel-strike
con.mode{3}.mode_num = p.modeNumFromGCboolean([0 1 1 0]');
con.mode{3}.x0.lb = [-inf; .8; repmat(-inf,32,1)];
con.mode{3}.x0.ub = [x0(1); repmat(inf,33,1)];
con.mode{3}.T.lb = 0;
con.mode{3}.T.ub = .2;
con.mode{3}.u.lb = repmat(umin,6,1);
con.mode{3}.u.ub = repmat(umax,6,1);

%{

% foot flat
con.mode{4}.mode_num = p.modeNumFromGCboolean([0 1 1 1]');
con.mode{4}.x0.lb = [repmat(-inf,16,1);x0(17); .8; repmat(-inf,size(x0(19:end)))];
con.mode{2}.x0.ub = repmat(inf,size(x0));
con.mode{4}.x0.ub = repmat(inf,size(x0));
con.mode{4}.T.lb = 0;
con.mode{4}.T.ub = .2;
con.mode{4}.u.lb = repmat(umin,6,1);
con.mode{4}.u.ub = repmat(umax,6,1);

% single support
con.mode{5}.mode_num = p.modeNumFromGCboolean([0 0 1 1]');
con.mode{5}.x0.lb = [repmat(-inf,16,1);x0(17); .8; repmat(-inf,size(x0(19:end)))];
con.mode{2}.x0.ub = repmat(inf,size(x0));
con.mode{5}.x0.ub = repmat(inf,size(x0));
con.mode{5}.T.lb = 0;
con.mode{5}.T.ub = .5;
con.mode{5}.u.lb = repmat(umin,6,1);
con.mode{5}.u.ub = repmat(umax,6,1);

% final conditions mirror initial conditions, with x values translated
% by>.4
minsteplength=.4;
maxsteplength=1;
con.mode{5}.xf.lb = [zeros(8,1);x0(1);x0(2)+minsteplength;x0(3:5);x0(6)+minsteplength;x0(7:8);x0(17)+minsteplength;x0(18:22);x0(29:34);x0(23:28)];
con.mode{5}.xf.ub = [zeros(8,1);x0(1);x0(2)+maxsteplength;x0(3:5);x0(6)+maxsteplength;x0(7:8);x0(17)+maxsteplength;x0(18:22);x0(29:34);x0(23:28)];
%}

options.method='dircol';
options.grad_method='user_then_numerical';

options.MajorOptimalityTolerance=1e-2;
options.MajorFeasibilityTolerance=1e-2;
options.MinorFeasibilityTolerance=1e-2;
options.SuperbasicsLimit=300;
a=@(t,x,u)plotDircolTraj(t,x,u,[17 18]);  % for debugging
%options.trajectory_cost_fun={a,[],[],[],[]};
%options.xtape0='simulate';
tic
%options.grad_test = true;
costfun = {@cost,@cost,@cost};%,@cost,@cost};
finalcostfun = {@finalcost,@finalcost,@finalcost};%,@finalcost,@finalcost};
x0m = {x0,x0,x0};%,x0,x0};

[utraj,xtraj,info] = trajectoryOptimization(p,costfun,finalcostfun,x0m,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

if (nargout<1)
  v = SpringFlamingoVisualizer();
  v.display_dt = .001;
  v.playback_speed = .02;
  playback(v,xtraj);
end

end



      function [g,dg] = cost(t,x,u);
        R = eye(6);
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
      end
      
      function [h,dh] = finalcost(t,x)
        h=0;
        dh = [0,zeros(1,size(x,1))];
      end
      
      function [J,dJ]=plotDircolTraj(t,x,u,plotdims)
        figure(25);
        h=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
        drawnow;
        delete(h);
        J=0;
        dJ=[0*t(:);0*x(:);0*u(:)]';
      end
