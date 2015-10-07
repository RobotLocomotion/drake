function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = constrainedTrajOpt
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
options.use_new_kinsol = true;

p = RigidBodyManipulator('OneLegHopper.urdf',options);

v = p.constructVisualizer();

N = [5,5,5];
T = {[.05 .4],[.05 .4],[.05 .1]};
modes = {[1;2;3;4],[3;4],[]};

% N = [5;5];
% T = {[0.2 .4],[.02 .1]};
% modes = {[1;2;3;4],[3;4]};

% N = 5;
% T = {[.2 .4]};
% modes = {[1;2;3;4]};

to_options = struct();
to_options.mode_options{1}.friction_limits = false;
to_options.mode_options{2}.friction_limits = false;
to_options.mode_options{3}.friction_limits = false;

% traj_opt=ConstrainedHybridTrajectoryOptimization(p,{[1;2]},N,T,to_options);
% traj_opt=ConstrainedHybridTrajectoryOptimization(p,{[1;2],[2]},N,T,to_options);
traj_opt=ConstrainedHybridTrajectoryOptimization(p,modes,N,T,to_options);

x0_min = [0;0;-inf;0;0;0;-inf(2,1);zeros(8,1)];
x0_max = [0;0;.2;0;inf;0;0;inf;zeros(8,1)];

xf_min = [-inf;0;7;0;-inf;0;-inf(2,1);-inf(8,1)];
xf_max = [inf;0;inf;0;inf;0;inf(2,1);inf(8,1)];

x0_nom = [0;0;.2;0;1;0;-2;2.5;zeros(8,1)];

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0_min,x0_max),1);

if length(modes) > 2
%   traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(xf_min,xf_max),N(end));
end

% traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(0,pi),1:N(1),5);

for i=1:length(modes)
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint([0;0;0],[0;0;0]),1:N(i),[2;4;6]);
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(-1,2),1:N(i),5);
end

traj_opt = traj_opt.addModeRunningCost(1,@running_cost_fun);

% 
t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},repmat(x0_nom,1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(2,N(1))));
% traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},randn(4,N(1))));

if length(modes) > 1
  t_init{2} = linspace(0,.4,N(2));
  traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},randn(16,N(2))));
  traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(2,N(2))));
%   traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},randn(2,N(2))));
  traj_opt = traj_opt.addModeStateConstraint(2,BoundingBoxConstraint(0,pi),1:N(2),8);
  traj_opt = traj_opt.addModeRunningCost(2,@running_cost_fun);
end

if length(modes) >2
  t_init{3} = linspace(0,.4,N(3));
  traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},randn(16,N(3))));
  traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(2,N(3))));
  traj_opt = traj_opt.addModeStateConstraint(3,BoundingBoxConstraint(0,pi),1:N(3),8);
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
    df = [0 zeros(1,16) 2*u'*R];
  end

end