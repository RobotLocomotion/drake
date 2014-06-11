function [p,xtraj,utraj,ltraj,z,F,info,traj_opt] = testNewTrajOpt
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
p = PlanarRigidBodyManipulator('../KneedCompassGait.urdf',options);
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);


N = 20;
T = 3;
T0 = 3;

initial_cost = [];final_cost = [];
running_cost = NonlinearConstraint(-inf,inf,16,@(z) running_cost_fun(z(1),z(2:13),z(14:16)));

t_init = linspace(0,T0,N);

% x0 = [0;0;1;zeros(15,1)];
% xf = [0;0;1;zeros(15,1)];
x0 = [0;1;zeros(10,1)];
xf = [.2;1;zeros(10,1)];

N2 = floor(N/2);

if nargin < 2
  traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
  traj_init.u = PPTrajectory(foh(t_init,randn(3,N)));
  traj_init.l = PPTrajectory(foh(t_init,T0/(N-1)*[repmat([1;zeros(7,1)],1,N2) repmat([zeros(4,1);1;zeros(3,1)],1,N-N2)]));
else
  traj_init.x = xtraj;
  traj_init.u = utraj;
end
T_span = [T T];



initial_constraint.mgr = ConstraintManager(LinearConstraint(x0,x0,[eye(12)]));
initial_constraint.i = {1};
final_constraint.mgr = ConstraintManager(LinearConstraint(xf,xf,[eye(12)]));
final_constraint.i = {N};

constraints = {initial_constraint, final_constraint};
to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.compl_slack = .1;

traj_opt = ContactImplicitTrajectoryOptimization(p,initial_cost,running_cost,final_cost,N,T_span,constraints,to_options);
% traj_opt = traj_opt.setCheckGrad(true);
snprint('snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
 [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);

function [f,df] = running_cost_fun(h,x,u)
  f = u'*u;
  df = [0 zeros(1,12) 2*u'];
end

end
