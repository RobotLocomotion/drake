function [p,xtraj,utraj,z,F,info,traj_opt] = testTrajOpt(xtraj,utraj)
p = PlanarRigidBodyManipulator('../Acrobot.urdf');
p = p.setInputLimits(-10,10);
% p = AcrobotPlant();

N = 21;
T = 6;
T0 = 4;


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

options = struct();
% options.integration_method = DirtranTrajectoryOptimization.FORWARD_EULER;
% options.integration_method = DirtranTrajectoryOptimization.BACKWARD_EULER;
% traj_opt = DirtranTrajectoryOptimization(p,N,T_span,options);
traj_opt = DircolTrajectoryOptimization(p,N,T_span,struct());
% traj_opt = traj_opt.setCheckGrad(true);
% snprint('snopt.out');
% snsummary('snsummary.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
traj_opt = traj_opt.addFinalCost(@final_cost_fun);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);
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

function [f,df] = final_cost_fun(T,x)
  K = 10;
  f = K*T;
  df = [K zeros(1,4)];
end