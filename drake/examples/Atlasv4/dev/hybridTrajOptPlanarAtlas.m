function [xtraj,utraj,ltraj,z,F,info,traj_opt] = hybridTrajOptPlanarAtlas(p,xtraj,utraj,ltraj,ljltraj,options)

if isfield(options,'mode_seq_vec')
  mode_seq_vec = options.mode_seq_vec;
else
  error('Need a mode sequence');
end

if isfield(options,'N_vec')
  N_vec = options.N_vec;  
else
  error('Need a N sequence');
end

N = sum(N_vec)+1;


if isfield(options,'t_init'),
  t_init = options.t_init;
  if length(t_init) ~= N
    error('If specified t_init should match N');
  end
else
  t_init = xtraj.pp.breaks;
  if length(t_init) ~= N
    t_init = linspace(0,t_init(end),N);
  end
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
  %loosened this delta
  x0_delta = [0;.2;.2;1;1;1;1;.2;.2;1;0.1;5*ones(9,1)];
end

if isfield(options,'xf_delta')
  typecheck(options.xf_delta,'double');
  sizecheck(options.xf_delta,[nx,1]);
  xf_delta = options.xf_delta;
else
  xf_delta = [0;0;0;1;1;1;1;.2;.2;1;0.1;5*ones(9,1)];
  % commented out some contraints, loosened this delta
  xf_delta = [0;.5;.5;1;1;1;1;.5;.5;1;0.5;5*ones(9,1)];
end

x0_min = x0 - x0_delta;
x0_max = x0 + x0_delta;

xf_min = xf - xf_delta;
xf_max = xf + xf_delta;

to_options.lambda_bound = 1e10;
% % to_options.lambda_jl_mult = tf0/N;

for i=1:size(mode_seq_vec,2)
  mode_seq{i} = find(mode_seq_vec(:,i));
end
T = options.t_span_mode;
traj_opt=ConstrainedHybridTrajectoryOptimization(p,mode_seq,N_vec,T,to_options);

for i=1:length(mode_seq)  
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@foot_height_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@pelvis_motion_fun);
  traj_opt = traj_opt.addModeFinalCost(i,@final_cost_fun);
end


traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0_min,x0_max),1);
% traj_opt = traj_opt.addModeStateConstraint(length(mode_seq),BoundingBoxConstraint(xf_min,xf_max),N_vec(end));

if isfield(options,'R_periodic') 
  typecheck(options.R_periodic,'double');
  sizecheck(options.R_periodic,[nx,2*nx]);
  R_periodic = options.R_periodic;
  periodic_constraint = LinearConstraint(zeros(nx,1),zeros(nx,1),R_periodic);
  periodic_inds = [traj_opt.mode_opt{1}.x_inds(:,1) + traj_opt.var_offset(1);...
    traj_opt.mode_opt{end}.x_inds(:,end) + traj_opt.var_offset(end)];
%   traj_opt = traj_opt.addConstraint(periodic_constraint,periodic_inds);
end

% for i=1:length(mode_seq)  
%   lz_inds = traj_opt.mode_opt{i}.l_inds(2:2:end,:) + traj_opt.var_offset(i);
%   for j=1:size(lz_inds,1),
%     contactDeltaCost = FunctionHandleConstraint(-inf,inf,size(lz_inds,2),@contact_delta_cost_fun,1);
%     traj_opt = traj_opt.addCost(contactDeltaCost,lz_inds(j,:)');
%   end
% end

% d = floor(N/8);
% lz_bound = BoundingBoxConstraint(zeros(2*floor(N/2) + 4*d,1),zeros(2*floor(N/2) + 4*d,1));
% lz_bound_inds = [lz_inds(3:4,1:floor(N/2)) lz_inds(1:2,N+1-2*d:end)];
% traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));
% 
% % lz_bound = BoundingBoxConstraint(zeros(2*(N - d),1),zeros(2*(N - d),1));
% % lz_bound_inds = [lz_inds(3:4,1:floor(N/2)-d) lz_inds(1:2,floor(N/2)+1:end)];
% % traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));
% 
% lz_bound = BoundingBoxConstraint(.2*ones(2*(N - 2*d),1),inf(2*(N - 2*d),1));
% lz_bound_inds = [lz_inds(1:2,1:floor(N/2)-d) lz_inds(3:4,floor(N/2)+d+1:end)];
% traj_opt = traj_opt.addBoundingBoxConstraint(lz_bound, lz_bound_inds(:));

for i=1:length(mode_seq)
  knee_inds = traj_opt.mode_opt{i}.x_inds([5;9],:) + traj_opt.var_offset(i);
  knee_inds = knee_inds(:);
  knee_constraint = BoundingBoxConstraint(.1*ones(length(knee_inds),1),inf(length(knee_inds),1));
%   traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);
  
  % bound joint velocities
  joint_vel_max = 5;
  joint_vel_bound = BoundingBoxConstraint(-joint_vel_max*ones(p.getNumVelocities,1),joint_vel_max*ones(p.getNumVelocities,1));
%   traj_opt = traj_opt.addModeStateConstraint(i,joint_vel_bound,1:N_vec(i),11:20);
end



traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',300);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',5000000);
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',5000);
% traj_opt = traj_opt.setSolverOptions('snopt','ScaleOption',1);
% traj_opt = traj_opt.setSolverOptions('snopt','LUSingularityTolerance',1e-6);
traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-5);
% traj_opt = traj_opt.setCheckGrad(true);
% z0 = traj_opt.getInitialVars(t_init,traj_init);
% [f,G] = traj_opt.objectiveAndNonlinearConstraints(z0);

for i=1:length(mode_seq)
  t_init_mode{i} = t_init(sum(N_vec(1:i-1)) + (1:N_vec(i)));
  traj_init_mode.mode{i}.x = PPTrajectory(foh(t_init_mode{i} - t_init_mode{i}(1),traj_init.x.eval(t_init_mode{i})));
  traj_init_mode.mode{i}.u = PPTrajectory(foh(t_init_mode{i} - t_init_mode{i}(1),traj_init.u.eval(t_init_mode{i})));
  
  if ~isempty(mode_seq{i})
    l_trans = [zeros(2,4*(mode_seq{i}(1)-1)) [0 1 -1 0;1 0 0 0] zeros(2,4*(4-mode_seq{i}(1)))];
    for j=2:length(mode_seq{i}),
      l_trans = [l_trans;[zeros(2,4*(mode_seq{i}(j)-1)) [0 1 -1 0;1 0 0 0] zeros(2,4*(4-mode_seq{i}(j)))]];
    end
    l_trans = l_trans*(length(ltraj.pp.breaks)/ltraj.tspan(2));
  traj_init_mode.mode{i}.l = PPTrajectory(foh(t_init_mode{i} - t_init_mode{i}(1),l_trans*traj_init.l.eval(t_init_mode{i})));
  end
  t_init_mode{i} = t_init_mode{i} - t_init_mode{i}(1);
end

traj_opt = traj_opt.compile();
if false  
  data=load('atlas_hybrid_dircol_4');
  for i=1:4,
    traj_init_mode.mode{i}.x = data.xtraj{i};
    traj_init_mode.mode{i}.u = data.utraj{i};
    traj_init_mode.mode{i}.l = data.ltraj{i};
    t_init_mode{i} = data.xtraj{i}.pp.breaks;
  end
end

if false
data = load('atlas_hybrid_dircol_4');
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(data.z);
else
[xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init_mode,traj_init_mode);
end
  function [f,df] = contact_delta_cost_fun(l)
    K = 200;
    f = K*sum(diff(l).^2);
    n = length(l);
    R = sparse([1:n-1 2:n], [1:n-1 1:n-1], [ones(n-1,1);-ones(n-1,1)]);
    df = -2*K*diff(l)'*R';
  end

  function [f,df] = running_cost_fun(h,x,u)
    K = .1;
    R = eye(nu);
    f = K*u'*R*u;
    df = [0 zeros(1,nx) 2*K*u'*R];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:nq);
    
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',true));
    phi0 = [.1;.1;.1;.1];
    K = 5;
    I = find(phi < phi0);
    f = K*(phi(I) - phi0(I))'*(phi(I) - phi0(I));
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi(I)-phi0(I))'*n(I,:) zeros(1,nv+nu)];
    
%     K = 500;
%     f = K*sum(phi0(I) - phi(I));
%     df = [0 K*sum(n(I,:),1) zeros(1,nv+nu)];

%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    pitch_idx = 3;
    pitch_dot_idx = p.getNumPositions+pitch_idx;

    Kq = 5;
    Kqd = 5;
    f = Kq*x(pitch_idx)^2 + Kqd*x(pitch_dot_idx)^2; % try to avoid moving the pelvis quickly
    df = zeros(1,1+nx+nu);
    df(1+pitch_idx) = 2*Kq*x(pitch_idx);
    df(1+pitch_dot_idx) = 2*Kqd*x(pitch_dot_idx);

%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end



  function [f,df] = final_cost_fun(T,x)
    K = 10;
    f = K*T;
    df = [K zeros(1,nx)];
    
  end

end
