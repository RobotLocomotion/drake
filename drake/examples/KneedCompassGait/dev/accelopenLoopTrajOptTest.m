function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = accelopenLoopTrajOptTest(N, T)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;
p = PlanarRigidBodyManipulator('../KneedCompassGait.urdf',options);
p = p.setJointLimits(-inf(6,1),inf(6,1));
p = p.compile();
v = p.constructVisualizer();
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);

%todo: add joint limits, periodicity constraint

mode = 1;


% x0 = [0;0;1;zeros(15,1)];
% xf = [0;0;1;zeros(15,1)];
x0 = [0;1;zeros(4,1);0*ones(6,1)];

  
x0_min = x0;
x0_max = x0;

to_options.test_bound = true;
to_options.friction_limits = false;
to_options.non_penetration = false;
to_options.relative_constraints = false;

traj_opt=ContactConstrainedDircolTrajectoryOptimization(p,N,[T T],mode,to_options);

t_init = linspace(0,T,N);
u = 1*ones(3,length(t_init));%40*cos(20*t_init);
traj_init.u = PPTrajectory(foh(t_init,u));
traj_init.u = traj_init.u.setOutputFrame(p.getInputFrame);
traj_opt = traj_opt.addConstraint(ConstantConstraint(u),traj_opt.u_inds);


traj_init.x = traj_init.u.cascade(traj_opt.plant).simulate([0 T],x0);

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);

traj_opt = traj_opt.setSolverOptions('snopt','ScaleOption',1);
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',400000);
traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',300000);


% traj_opt = traj_opt.setCheckGrad(true);

[xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end
