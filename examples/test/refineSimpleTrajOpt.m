function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = refineSimpleTrajOpt(x0,xf,T,xtraj,utraj,ltraj,ljltraj,scale)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = false;
options.ignore_self_collisions = true;
options.use_bullet = false;
p = PlanarRigidBodyManipulator('SimpleHopper.urdf',options);
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);

%todo: add joint limits, periodicity constraint

N = 10;
  
traj_init.x = xtraj;
traj_init.u = utraj;
traj_init.l = ltraj;
traj_init.ljl = ljltraj;


T_span = [T T];

t_init = linspace(0,T,N);


x0_min = x0;
x0_max = x0;

xf_min = xf;
xf_max = xf;

to_options.compl_slack = scale*.01;
to_options.lincompl_slack = scale*.001;
to_options.jlcompl_slack = scale*.01;

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.lambda_mult = p.getMass*9.81*T/N/2;
to_options.lambda_jl_mult = T/N;

to_options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.BACKWARD_EULER;

traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
% traj_opt = traj_opt.addRunningCost(@foot_height_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
% traj_opt = traj_opt.addInputConstraint(LinearConstraint(zeros(2,1),zeros(2,1),[eye(2),-eye(2)]),{[1,N-1]});% force first and next-to-last inputs to be equal (we drop the last input because it's junk)

% traj_opt = traj_opt.setCheckGrad(true);
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',5000);
traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-5);
[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);


  function [f,df] = running_cost_fun(h,x,u)
    K = 10;
    R = eye(getNumInputs(p));
    f = K*u'*R*u;
    df = [0 zeros(1,6) 2*K*u'*R];
  end
end
