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

N = 30;
T = .4;

to_options.lambda_bound = 1e3;

traj_opt = ContactConstrainedDircolTrajectoryOptimization(p,N,[.1 T],[1;2],to_options);

x0_min = [-inf(5,1);zeros(5,1)];
x0_max = [inf;.2;inf(3,1);zeros(5,1)];

xf_min = [-inf;.5;-inf(3,1);zeros(5,1)];
xf_max = [inf(5,1);zeros(5,1)];

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(0,inf),1:N,5);

t_init = linspace(0,T,N);
traj_init.x = PPTrajectory(foh(t_init,randn(10,N)));
traj_init.u = PPTrajectory(foh(t_init,randn(2,N)));
traj_init.l = PPTrajectory(foh(t_init,randn(4,N)));

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',500000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);

[xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
