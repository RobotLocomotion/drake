function [p,v,xtraj,utraj,z,F,info,traj_opt] = testConstrainedDircol

p = PlanarRigidBodyManipulator('FourBar.urdf');
v = p.constructVisualizer();
v.xlim = [-8 8]; v.ylim = [-4 10];
N = 20;
T = 1;

x0 = p.getInitialState;
x0(4:6) = 0;

traj_opt = ConstrainedDircolTrajectoryOptimization(p,N,[T T]);

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0,x0),1);

xf_min = [-inf;-inf;2;-inf(3,1)];
xf_max = [inf;inf;4;inf(3,1)];
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);


t_init = linspace(0,T,N);
traj_init.x = PPTrajectory(foh(t_init,repmat(x0,1,N)));
% traj_init.u = PPTrajectory(foh(t_init,zeros(0,N)));
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',500000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);
% traj_opt = traj_opt.setCheckGrad(true);

traj_opt = traj_opt.addRunningCost(@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);

[xtraj,utraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);



  function [f,df] = running_cost_fun(h,x,u)
    R = 1;
    f = u'*R*u;
    df = [0 zeros(1,6) 2*u'*R];
  end


  function [f,df] = final_cost_fun(T,x)
    K = 100;
    f = K*x(3)^2;
    df = [0 0 0 2*K*x(3) 0 0 0];    
  end



end