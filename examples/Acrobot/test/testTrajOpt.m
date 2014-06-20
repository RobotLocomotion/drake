function [p,xtraj,utraj,z,F,info,traj_opt] = testTrajOpt(xtraj,utraj)
p = PlanarRigidBodyManipulator('../Acrobot.urdf');
p = p.setInputLimits(-10,10);
% p = AcrobotPlant();

N = 21;
T = 6;
T0 = 4;

final_cost = NonlinearObjective(-inf,inf,N+3,@final_cost_fun);
running_cost = NonlinearObjective(-inf,inf,6,@running_cost_fun);

x0 = zeros(4,1);
xf = [pi;0;0;0];

t_init = linspace(0,T0,N);

if nargin < 2
  traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
  traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
  traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
  traj_init.u = traj_init.u.setOutputFrame(p.getInputFrame);
else
  traj_init.x = xtraj;
  traj_init.u = utraj;
end
T_span = [T/2 T];


% traj_opt = DirtranTrajectoryOptimization(DirtranTrajectoryOptimization.MIDPOINT,p,N,T_span,struct());
% traj_opt = DirtranTrajectoryOptimization(DirtranTrajectoryOptimization.FORWARD_EULER,p,N,T_span,struct());
% traj_opt = DirtranTrajectoryOptimization(DirtranTrajectoryOptimization.BACKWARD_EULER,p,N,T_span,struct());
traj_opt = DircolTrajectoryOptimization(p,N,T_span,struct());
% traj_opt = traj_opt.setCheckGrad(true);
% snprint('snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.addRunningCost(running_cost);
traj_opt = traj_opt.addFinalCost(final_cost);
traj_opt = traj_opt.addStateConstraint(EqualityConstraint(x0),1);
traj_opt = traj_opt.addStateConstraint(EqualityConstraint(xf),N);
% traj_opt = traj_opt.addLinearStateConstraint(LinearConstraint(xf,xf,eye(4)),N);

tic
[xtraj,utraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
toc


v = p.constructVisualizer();
v.playback(xtraj);

% keyboard
end

function [f,df] = running_cost_fun(h,x,u)
  f = u^2;
  df = [0 zeros(1,4) 2*u];
end

function [f,df] = final_cost_fun(h,x)
  K = 1;
  f = K*sum(h);
  df = [K*ones(1,length(h)) zeros(1,4)];
end