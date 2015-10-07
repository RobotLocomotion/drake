function [xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = trajOptPlanarAtlas(p,xtraj,utraj,ltraj,ljltraj,options)

if isfield(options,'N')
  typecheck(options.N,'double');
  sizecheck(options.N,1);
  N = options.N;
else
  N = 21;
end

if isfield(options,'scale')
  typecheck(options.scale,'double');
  sizecheck(options.scale,1);
  scale = options.scale;
else
  scale = 1;
end

t_init = xtraj.pp.breaks;
if length(t_init) ~= N
  t_init = linspace(0,t_init(end),N);
end
traj_init.x = xtraj;
traj_init.u = utraj;
traj_init.l = ltraj;
traj_init.ljl = ljltraj;

if isfield(options,'T_span')
  typecheck(options.T_span,'double');
  sizecheck(options.T_span,[1,2]);
  T_span = options.T_span;
  tf0 = T_span(2);
else
  tf0 = t_init(end);
  T_span = [tf0/2,tf0];
end

nx = p.getNumStates();
nu = p.getNumInputs();
nq = p.getNumPositions();
nv = p.getNumVelocities();

if isfield(options,'x0')
  typecheck(options.x0,'double');
  sizecheck(options.x0,[nx,1]);
  x0 = options.x0;
else
  x0 = xtraj.eval(xtraj.tspan(1));
end

if isfield(options,'xf')
  typecheck(options.xf,'double');
  sizecheck(options.xf,[nx,1]);
  xf = options.xf;
else
  xf = xtraj.eval(xtraj.tspan(2));
end

if isfield(options,'x0_delta')
  typecheck(options.x0_delta,'double');
  sizecheck(options.x0_delta,[nx,1]);
  x0_delta = options.x0_delta;
else
  x0_delta = [0;0;0;1;1;1;1;.2;.2;1;0.1;5*ones(9,1)];
end

if isfield(options,'xf_delta')
  typecheck(options.xf_delta,'double');
  sizecheck(options.xf_delta,[nx,1]);
  xf_delta = options.xf_delta;
else
  xf_delta = [0;0;0;1;1;1;1;.2;.2;1;0.1;5*ones(9,1)];
end

x0_min = x0 - x0_delta;
x0_max = x0 + x0_delta;

xf_min = xf - xf_delta;
xf_max = xf + xf_delta;

to_options.compl_slack = scale*.01;
to_options.lincompl_slack = scale*.01;
to_options.jlcompl_slack = scale*.0001;

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.lambda_mult = p.getMass*9.81*T_span(2)/N/2;
to_options.lambda_jl_mult = tf0/N;

to_options.allow_sliding = false;

% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;
to_options.integration_method = ContactImplicitTrajectoryOptimization.BACKWARD_EULER;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;

traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
traj_opt = traj_opt.addRunningCost(@foot_height_fun);
traj_opt = traj_opt.addRunningCost(@pelvis_motion_fun);
traj_opt = traj_opt.addRunningCost(@ankle_pitch_fun);
traj_opt = traj_opt.addFinalCost(@final_cost_fun);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
if isfield(options,'R_periodic') 
  typecheck(options.R_periodic,'double');
  sizecheck(options.R_periodic,[nx,2*nx]);
  R_periodic = options.R_periodic;
  periodic_constraint = LinearConstraint(zeros(nx,1),zeros(nx,1),R_periodic);
  traj_opt = traj_opt.addStateConstraint(periodic_constraint,{[1 N]});
end
lz_inds = traj_opt.l_inds(1:4:end,:);
contactDeltaCost = FunctionHandleConstraint(-inf,inf,size(lz_inds,2),@contact_delta_cost_fun,1);
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(1,:)');
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(2,:)');
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(3,:)');
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(4,:)');

d = floor(N/8);
lz_bound = BoundingBoxConstraint(zeros(2*floor(N/2) + 4*d,1),zeros(2*floor(N/2) + 4*d,1));
lz_bound_inds = [lz_inds(3:4,1:floor(N/2)) lz_inds(1:2,N+1-2*d:end)];
traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));

% lz_bound = BoundingBoxConstraint(zeros(2*(N - d),1),zeros(2*(N - d),1));
% lz_bound_inds = [lz_inds(3:4,1:floor(N/2)-d) lz_inds(1:2,floor(N/2)+1:end)];
% traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));

lz_bound = BoundingBoxConstraint(.2*ones(2*(N - 2*d),1),inf(2*(N - 2*d),1));
lz_bound_inds = [lz_inds(1:2,1:floor(N/2)-d) lz_inds(3:4,floor(N/2)+d+1:end)];
traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));

knee_inds = traj_opt.x_inds([5;9],:);
knee_inds = knee_inds(:);
knee_constraint = BoundingBoxConstraint(.1*ones(length(knee_inds),1),inf(length(knee_inds),1));
traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',500000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',5000);
traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-5);
% traj_opt = traj_opt.setCheckGrad(true);
z0 = traj_opt.getInitialVars(t_init,traj_init);
[f,G] = traj_opt.objectiveAndNonlinearConstraints(z0);
[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);

  function [f,df] = contact_delta_cost_fun(l)
    K = 200;
    f = K*sum(diff(l).^2);
    n = length(l);
    R = sparse([1:n-1 2:n], [1:n-1 1:n-1], [ones(n-1,1);-ones(n-1,1)]);
    df = -2*K*diff(l)'*R';
  end

  function [f,df] = running_cost_fun(h,x,u)
    K = 0.01;
    R = eye(nu);
    f = K*u'*R*u;
    df = [0 zeros(1,nx) 2*K*u'*R];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:nq);
    
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',true));
    phi0 = [.15;.15;.15;.15];
    K = 120;
    I = find(phi < phi0);
    f = K*(phi(I) - phi0(I))'*(phi(I) - phi0(I));
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi(I)-phi0(I))'*n(I,:) zeros(1,nv+nu)];

%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    pitch_idx = 3;
    pitch_dot_idx = p.getNumPositions+pitch_idx;

    Kq = 500;
    Kqd = 500;
    f = Kq*x(pitch_idx)^2 + Kqd*x(pitch_dot_idx)^2; % try to avoid moving the pelvis quickly
    df = zeros(1,1+nx+nu);
    df(1+pitch_idx) = 2*Kq*x(pitch_idx);
    df(1+pitch_dot_idx) = 2*Kqd*x(pitch_dot_idx);
  end

  function [f,df] = ankle_pitch_fun(h,x,u)
    pitch_idx = [6;10];
    pitch_dot_idx = p.getNumPositions+pitch_idx;

    Kq = 200;
    Kqd = 20;
    f = Kq*x(pitch_idx)'*x(pitch_idx) + Kqd*x(pitch_dot_idx)'*x(pitch_dot_idx); 
    df = zeros(1,1+nx+nu);
    df(1+pitch_idx) = 2*Kq*x(pitch_idx)';
    df(1+pitch_dot_idx) = 2*Kqd*x(pitch_dot_idx)';
  end



  function [f,df] = final_cost_fun(T,x)
    K = 1;
    f = K*T;
    df = [K zeros(1,nx)];
    
  end

end
