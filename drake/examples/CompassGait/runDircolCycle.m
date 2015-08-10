function [p,utraj,xtraj,z,traj_opt]=runDircolCycle
% from an initial balancing condition, take one step and return to the
% balance upright.

p = CompassGaitPlant();

N = [15;15];

options.u_const_across_transitions = true;
options.periodic = true;

traj_opt = HybridTrajectoryOptimization(@DircolTrajectoryOptimization, p,[1;2],N,{[.2 .5],[.2 .5]},options);

x0 = [0;0;2;-.4];
t1 = .417;
x1 = [-.326;.22;-.381;-1.1];
tf = .713;
xf = x0;

t_init{1} = linspace(0,t1,N(1));
t_init{2} = linspace(0,tf-t1,N(2));

traj_init{1}.x0 = x0;
traj_init{2}.x0 = x1;

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint([0;-inf(3,1)],[0;inf(3,1)]),1);
traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint([.1;-inf(3,1)],inf(4,1)),N(1));

traj_opt = traj_opt.addModeRunningCost(1,@cost);
traj_opt = traj_opt.addModeRunningCost(2,@cost);

traj_opt = traj_opt.compile();
% traj_opt = traj_opt.setCheckGrad(true);
% snprint('snopt.out');
tic
[xtraj,utraj,z,F,info] = solveTraj(traj_opt,t_init,traj_init);
toc
if (nargout<1)
  v = CompassGaitVisualizer(p);
  figure(1); clf;
  fnplt(utraj);
  
  figure(2); clf; hold on;
  fnplt(xtraj,[2 4]);
  fnplt(xtraj,[3 5]);
  
  playback(v,xtraj);
  
end
end

function [g,dg] = cost(t,x,u);
R = 1;
g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
end

function [h,dh] = finalcost(t,x)
h=t;
dh = [1,zeros(1,size(x,1))];
end

function J = postImpactTrajCost(T,X,U,p)
% encourage post-impact trajectory to leave collision surface orthogonally
t0=T(1); x0=X(:,1); u0=U(:,1);
xdot0=p.modes{2}.dynamics(t0,x0,u0);
dphidx = [1,1,0,0]; % gradient of foot collision guard 1 w/respect to x
J=100*abs(dphidx*xdot0)./norm(dphidx)./norm(xdot0);
end

