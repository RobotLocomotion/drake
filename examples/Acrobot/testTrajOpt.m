function [p,xtraj,utraj,z,F,info] = testTrajOpt(xtraj,utraj)
% p = PlanarRigidBodyManipulator('../Acrobot.urdf');
p = AcrobotPlant();
initial_cost = [];
final_cost = [];
running_cost = NonlinearConstraint(-inf,inf,6,@(z) running_cost_fun(z(1),z(2:5),z(6)));

N = 30;
T = 2;

t_init = linspace(0,T,N);

if nargin < 2
  traj_init.x = PPTrajectory(foh(t_init,zeros(4,N)));
  traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
else
  traj_init.x = xtraj;
  traj_init.u = utraj;
end
T_span = [T T];

x0 = zeros(4,1);
xf = [pi;0;0;0];

initial_constraint.mgr = ConstraintManager(LinearConstraint(x0,x0,[eye(4)]));
initial_constraint.i = {1};
final_constraint.mgr = ConstraintManager(LinearConstraint(xf,xf,[eye(4)]));
final_constraint.i = {N};

traj_opt = BEDirtranTrajectoryOptimization(p,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,initial_constraint,final_constraint,struct());
% traj_opt = MidpointTrajectoryOptimization(p,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,initial_constraint,final_constraint,struct());
% traj_opt = traj_opt.setCheckGrad(true);
snprint('snopt.out');
[xtraj,utraj,z,F,info] = traj_opt.solveTraj();

% keyboard
end

function [f,df] = running_cost_fun(h,x,u)
  f = u^2;
  df = [zeros(1,5) 2*u];
end