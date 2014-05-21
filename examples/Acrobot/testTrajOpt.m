function [p,xtraj,utraj,z,F,info] = testTrajOpt(xtraj,utraj)
p = PlanarRigidBodyManipulator('Acrobot.urdf');
% p = AcrobotPlant();

N = 30;
T = 5;
T0 = 2;

initial_cost = [];
% final_cost = NonlinearConstraint(-inf,inf,N+3,@(z) final_cost_fun(z(1:N-1),z(N:N+3)));
final_cost = [];
running_cost = NonlinearConstraint(-inf,inf,6,@(z) running_cost_fun(z(1),z(2:5),z(6)));



t_init = linspace(0,T0,N);

if nargin < 2
  traj_init.x = PPTrajectory(foh(t_init,zeros(4,N)));
  traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
else
  traj_init.x = xtraj;
  traj_init.u = utraj;
end
T_span = [T/2 T];

x0 = zeros(4,1);
xf = [pi;0;0;0];

initial_constraint.mgr = ConstraintManager(LinearConstraint(x0,x0,[eye(4)]));
initial_constraint.i = {1};
final_constraint.mgr = ConstraintManager(LinearConstraint(xf,xf,[eye(4)]));
final_constraint.i = {N};

constraints = {initial_constraint, final_constraint};
% traj_opt = DircolTrajectoryOptimization(p,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,constraints,struct());
% traj_opt = MidpointTrajectoryOptimization(p,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,constraints,struct());
% traj_opt = FEDirtranTrajectoryOptimization(p,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,constraints,struct());
traj_opt = BEDirtranTrajectoryOptimization(p,initial_cost,running_cost,final_cost,t_init,traj_init,T_span,constraints,struct());
% traj_opt = traj_opt.setCheckGrad(true);
snprint('snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
[xtraj,utraj,z,F,info] = traj_opt.solveTraj();

% keyboard
end

function [f,df] = running_cost_fun(h,x,u)
  f = h*u^2;
  df = [u^2 zeros(1,4) 2*u*h];
end

function [f,df] = final_cost_fun(h,x)
  K = 10;
  f = K*sum(h);
  df = [K*ones(1,length(h)) zeros(1,4)];
end