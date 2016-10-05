data = [];
do_plots = true;
p = ConstrainedPlant;
N = 30;
T = .5;

options.accel_cost = 1e-6;
options.constrain_start = false; % since x0 is fixed
traj_opt = AccelConstrainedDircolTrajectoryOptimization(p,N,[T T],options);

x0 = [0;0;1;2]; % for first model
l0 = 1;
t_init = linspace(0,T,N);

traj_init.x = PPTrajectory(foh(t_init,repmat(x0,1,N)));
traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
traj_init.l = PPTrajectory(foh(t_init,repmat(l0,1,N)));

traj = p.simulate([0 T],x0);
%%

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0,x0),1);
%     traj_opt = traj_opt.setCheckGrad(true);
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.addCost(QuadraticConstraint(0,0,eye(N),100*ones(N,1)),traj_opt.l_inds);

[xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);

%%
tt = xtraj.pp.breaks;
t = linspace(0,T,1e3);
x = xtraj.eval(t);
xsim = traj.eval(t);
xsim_pt = traj.eval(tt);
x_pt = xtraj.eval(tt);

phi = zeros(1,length(t));
for i=1:length(t),
  phi(i) = p.positionConstraints(x(1:2,i));
end


if do_plots
  figure(1)
  subplot(3,1,1)
  plot(t,x,t,xsim,tt,xtraj.eval(tt),'*-')
  xlim([0 T])
  ylabel('x(t)')
  subplot(3,1,3)
  plot(t,x-xsim,'-')
  ylabel('Integration Error')
  xlim([0 T])
  subplot(3,1,2)
  plot(t,phi)
  xlabel('Time')
  ylabel('Constraint violation')
  
%   figure(2)
%   subplot(2,1,1)
%   plot(tt,x_pt,'*',tt,xsim_pt,'*')
%   xlim([0 T])
%   ylabel('x(t)')
%   subplot(2,1,2)
%   plot(tt,x_pt-xsim_pt,'*')
%   ylabel('Error')
%   xlim([0 T])
%   
%   figure(3)
%   plot(tt,z(traj_opt.l_inds),'*',(tt(1:end-1)+tt(2:end))/2,z(traj_opt.lc_inds),'*')
end

%%
result.N = N;
result.tt = tt;
result.err = x_pt - xsim_pt;
result.err_sum = sqrt(sum((x_pt - xsim_pt).*(x_pt - xsim_pt)));
result.bound = options.lambda_bound;
result.info = info;
result.err_final = result.err_sum(end);
data = [data;result];
display(sprintf('N %d, info %d',N,info));