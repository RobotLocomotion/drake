function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = hybridHopperConstrainedTrajOpt
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
options.use_new_kinsol = true;

p = PlanarRigidBodyManipulator('OneLegHopper.urdf',options);

v = p.constructVisualizer();

N = [10,10,10];
T = {[.05 .4],[.05 .4],[.05 .1]};
modes = {[1;2],[2],[]};

% N = [5;5];
% T = {[0.01 .4],[.02 .1]};
% modes = {[1;2],[2]};

% N = 5;
% T = {[0 .4]};
% modes = {[1;2]};

to_options.lambda_bound = 1e3;

% traj_opt=ConstrainedHybridTrajectoryOptimization(p,{[1;2]},N,T,to_options);
% traj_opt=ConstrainedHybridTrajectoryOptimization(p,{[1;2],[2]},N,T,to_options);
traj_opt=ConstrainedHybridTrajectoryOptimization(p,modes,N,T,to_options);

x0_min = [0;-inf;0;-inf(2,1);zeros(5,1)];
x0_max = [0;.2;inf;0;inf;zeros(5,1)];

xf_min = [-inf;.7;-inf(3,1);-inf(5,1)];
xf_max = [inf(5,1);inf(5,1)];

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(xf_min,xf_max),N(end));

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(0,pi),1:N(1),5);

traj_opt = traj_opt.addModeRunningCost(1,@running_cost_fun);

% 
t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},randn(10,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(2,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},randn(4,N(1))));

if length(modes) > 1
  t_init{2} = linspace(0,.4,N(2));
  traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},randn(10,N(2))));
  traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(2,N(2))));
  traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},randn(2,N(2))));
  traj_opt = traj_opt.addModeStateConstraint(2,BoundingBoxConstraint(0,pi),1:N(2),5);
  traj_opt = traj_opt.addModeRunningCost(2,@running_cost_fun);
end

if length(modes) >2
  t_init{3} = linspace(0,.4,N(3));
  traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},randn(10,N(3))));
  traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(2,N(3))));
  traj_opt = traj_opt.addModeStateConstraint(3,BoundingBoxConstraint(0,pi),1:N(3),5);
  traj_opt = traj_opt.addModeRunningCost(3,@running_cost_fun);
end

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',500000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);


% t
% 

% traj_opt = traj_opt.setCheckGrad(true);

traj_opt = traj_opt.compile();
 
[xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);



  function [f,df] = running_cost_fun(h,x,u)
    R = 1;
    f = u'*R*u;
    df = [0 zeros(1,10) 2*u'*R];
  end

end