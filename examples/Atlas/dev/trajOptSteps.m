function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = trajOptSteps(xtraj,utraj,ltraj,ljltraj,scale,N)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');

l = 0.4;
h = 0.11;
boxes = [0.25+l, 0.0, 2*l, 1, h;
         0.25+l+l/2, 0.0, l, 1, 2*h];
options.terrain = RigidBodyStepTerrain(boxes);

options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
p = PlanarRigidBodyManipulator('../urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);

if nargin < 6
  N = 40;
end


v = p.constructVisualizer;
N1 = floor(N/2);
N2 = N-N1;
d = floor(N/8);
if nargin < 2
  dd=load('data/atlas_step_qtraj.mat');

  tf0 = dd.qtraj.tspan(2);
  t_init = linspace(0,tf0,N);
  q_vec = dd.qtraj.eval(t_init);
  x_vec = [q_vec; 0*q_vec];
  x0 = x_vec(:,1);
  xf = x_vec(:,N);
  traj_init.x = PPTrajectory(foh(t_init,x_vec));
  traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame); 
  traj_init.u = PPTrajectory(foh(t_init,0*randn(5,N)));
  
  lp = [1;0;0;0];
  ln = zeros(4,1);
  traj_init.l = PPTrajectory(foh(t_init,[repmat([lp;lp;ln;ln],1,N1-d) zeros(16,2*d) repmat([ln;ln;lp;lp],1,N2-d)]));
  traj_init.ljl = [];
  
  scale = 1;
else
  t_init = xtraj.pp.breaks;
  if length(t_init) ~= N
    t_init = linspace(0,t_init(end),N);
  end
  traj_init.x = xtraj;
  traj_init.u = utraj;
  traj_init.l = ltraj;
  traj_init.ljl = ljltraj;
  tf0 = t_init(end);
end
T_span = [tf0/2 tf0];


x0_min = x0 - [0;0;0;1;1;1;1;.2;.2;1;5*ones(10,1)];
x0_max = x0 + [0;0;0;1;1;1;1;.2;.2;1;5*ones(10,1)];

xf_min = xf - [0;0;0;1;1;1;1;1;1;1;5*ones(10,1)];
xf_max = xf + [0;0;0;1;1;1;1;1;1;1;5*ones(10,1)];

to_options.compl_slack = scale*.01;
to_options.lincompl_slack = scale*.01;
to_options.jlcompl_slack = scale*.0001;

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.lambda_mult = p.getMass*9.81*tf0/N/2;
to_options.lambda_jl_mult = tf0/N;

% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;
to_options.integration_method = ContactImplicitTrajectoryOptimization.BACKWARD_EULER;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;

traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
traj_opt = traj_opt.addRunningCost(@foot_height_fun);
traj_opt = traj_opt.addFinalCost(@final_cost_fun);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);

lz_inds = traj_opt.l_inds(1:4:end,:);
contactDeltaCost = FunctionHandleConstraint(-inf,inf,size(lz_inds,2),@contact_delta_cost_fun,1);
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(1,:)');
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(2,:)');
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(3,:)');
traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(4,:)');

lz_bound = BoundingBoxConstraint(zeros(2*N1 + 4*d,1),zeros(2*N1 + 4*d,1));
lz_bound_inds = [lz_inds(3:4,1:N1) lz_inds(1:2,N+1-2*d:end)];
traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));

% lz_bound = BoundingBoxConstraint(zeros(2*(N - d),1),zeros(2*(N - d),1));
% lz_bound_inds = [lz_inds(3:4,1:N1-d) lz_inds(1:2,N1+1:end)];
% traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));

lz_bound = BoundingBoxConstraint(.2*ones(2*(N - 2*d),1),inf(2*(N - 2*d),1));
lz_bound_inds = [lz_inds(1:2,1:N1-d) lz_inds(3:4,N1+d+1:end)];
traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));

knee_inds = traj_opt.x_inds([5;9],:);
knee_inds = knee_inds(:);
knee_constraint = BoundingBoxConstraint(.1*ones(length(knee_inds),1),inf(length(knee_inds),1));
traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);

snprint('snopt.out');
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
    R = eye(5);
    f = K*u'*R*u;
    df = [0 zeros(1,20) 2*K*u'*R];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:10);
    
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',true));
    phi0 = [.2;.2;.2;.2];
    K = 50;
    I = find(phi < phi0);
    f = K*(phi(I) - phi0(I))'*(phi(I) - phi0(I));
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi(I)-phi0(I))'*n(I,:) zeros(1,15)];

%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

  function [f,df] = final_cost_fun(T,x)
    K = 1;
    f = K*T;
    df = [K zeros(1,20)];
    
  end

end
