function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = trajOpt(xtraj,utraj,ltraj,ljltraj,scale,t_init)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
p = PlanarRigidBodyManipulator('OneLegHopper.urdf',options);
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);

%todo: add joint limits, periodicity constraint

if nargin <6
  N = 40;
else
  N = length(t_init);
end

distance = .1;
x_vel = -0.0;

qd_init = [x_vel;zeros(4,1)];

q0 = [0;0;.6;-1.2;.6+pi/2];
phi_f = p.contactConstraints(q0);
q0(2) = -phi_f(1);
x0 = [q0;qd_init];

q1 = [-distance/2;0;.6;-1.2;.2+pi/2];
phi_f = p.contactConstraints(q1);
q1(2) = -phi_f(1) + 0.15;
x1 = [q1;qd_init];

qf = [-distance;0;.6;-1.2;.6+pi/2];
phi_f = p.contactConstraints(qf);
qf(2) = -phi_f(1);
xf = [qf;qd_init];

N1 = floor(N/2);
N2 = N-N1;
d = floor(N/4);
tf0 = 0.5;
if nargin < 2  
  t_init = linspace(0,tf0,N);
  traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,floor(N/2)),linspacevec(x1,xf,N-floor(N/2))]));
  traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
 
  traj_init.u = PPTrajectory(foh(t_init,zeros(getNumInputs(p),N)));
  
  lp = [1;0;0;0];
  ln = zeros(4,1);
  traj_init.l = PPTrajectory(foh(t_init,[repmat([lp;lp],1,d) repmat([ln;ln],1,N-2*d) repmat([lp;lp],1,d)]));
  traj_init.ljl = [];
  
  scale = 1;
else
  if nargin < 6
    t_init = xtraj.pp.breaks;      
    if length(t_init) ~= N
      t_init = linspace(0,t_init(end),N);
    end
  else
    to_options.time_option = 3;
    to_options.time_scaling = 1./diff(t_init);
  end
  
  traj_init.x = xtraj;
  traj_init.u = utraj;
  traj_init.l = ltraj;
  traj_init.ljl = ljltraj;
end


T_span = [tf0 tf0];


x0_min = [q0;qd_init];
x0_max = [q0;qd_init];

xf_min = [qf;qd_init] - [.0;zeros(9,1)];
xf_max = [qf;qd_init] + [.0;zeros(9,1)];

to_options.compl_slack = scale*.01;
to_options.lincompl_slack = scale*.001;
to_options.jlcompl_slack = scale*.01;

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.lambda_mult = p.getMass*9.81*tf0/N/2;
to_options.lambda_jl_mult = tf0/N;

to_options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.BACKWARD_EULER;

traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
% traj_opt = traj_opt.addRunningCost(@foot_height_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
traj_opt = traj_opt.addInputConstraint(LinearConstraint(zeros(2,1),zeros(2,1),[eye(2),-eye(2)]),{[1,N-1]});% force first and next-to-last inputs to be equal (we drop the last input because it's junk)

% traj_opt = traj_opt.setCheckGrad(true);
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',5000);
traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-5);
[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);

  function [f,df] = contact_delta_cost_fun(l)
    K = 500;
    f = K*sum(diff(l).^2);
    n = length(l);
    R = sparse([1:n-1 2:n], [1:n-1 1:n-1], [ones(n-1,1);-ones(n-1,1)]);
    df = -2*K*diff(l)'*R';
  end

  function [f,df] = running_cost_fun(h,x,u)
    K = 10;
    R = eye(getNumInputs(p));
    f = K*u'*R*u;
    df = [0 zeros(1,10) 2*K*u'*R];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:9);
    K = 50;
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',false));
    phi0 = [.1;.2;.1;.2];
    f = K*(phi - phi0)'*(phi - phi0);
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi-phi0)'*n zeros(1,13)];
  end

  function [f,df] = final_cost_fun(T,x)
    K = 1;
    f = K*T;
    df = [K zeros(1,10)];
    
  end

end
