function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = trajOptStopping(x0,z0,xtraj,utraj,ltraj)

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;

p = RigidBodyManipulator('../urdf/atlas_simple_contact_noback.urdf',options);
nq = p.getNumPositions;
nv = p.getNumVelocities;
nu = p.getNumInputs;
nx = p.getNumStates();
v = p.constructVisualizer();

N = [5,4,3];

duration = {[.2 .7],[.05 .4], [.05 .2]};

modes = {[5;6;7;8],[1;2;3;4;7;8],[1;2;3;4;5;6;7;8]};


to_options = struct();
to_options.mode_options{1}.active_inds = [1;2;3;4;6;9];
to_options.mode_options{2}.active_inds = [1;2;3;4;6;9;13;14;15;16;18];
to_options.mode_options{3}.active_inds = [1;2;3;4;6;9;13;14;15;16;18;21];

to_options.mode_options{1}.accel_cost = .001;
to_options.mode_options{2}.accel_cost = .001;
to_options.mode_options{3}.accel_cost = .001;

traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);

l0 = [0;0;897.3515;0;0;897.3515;0;0;179.1489;0;0;179.1489]/2;

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;.001*ones(17,1);.01*ones(18,1)],x0+[0;.001*ones(17,1);01*ones(18,1)]),1);

traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(zeros(18,1),zeros(18,1)),N(end)-1,19:36);
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(zeros(18,1),zeros(18,1)),N(end),19:36);

t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},repmat(x0,1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(nu,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},repmat(l0,1,N(1))));

if length(N) > 1
  t_init{2} = linspace(0,.2,N(2));
  traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},repmat(x0,1,N(2))));
  traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(nu,N(2))));
  traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([l0;l0(1:6)],1,N(2))));
end
if length(N) > 2
  t_init{3} = linspace(0,.2,N(3));
  traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},repmat(x0,1,N(3))));
  traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(nu,N(3))));
  traj_init.mode{3}.l = PPTrajectory(foh(t_init{3},repmat([l0;l0],1,N(3))));
end

if nargin > 2
  for i=1:length(N)
    t_init{i} = linspace(0,diff(xtraj{i}.tspan),N(i));
    traj_init.mode{i}.x = xtraj{i}.shiftTime(-xtraj{i}.tspan(1));
    if nargin < 2
      traj_init.mode{i}.u = utraj{i}.shiftTime(-utraj{i}.tspan(1));
    end
    if nargin > 3
      traj_init.mode{i}.l = ltraj{i}.shiftTime(-ltraj{i}.tspan(1));
    end
  end
end

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);



% traj_opt = traj_opt.addModeRunningCost(1,@foot_height_fun);
% traj_opt = traj_opt.setCheckGrad(true);
for i=1:length(N)
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@pelvis_motion_fun);
  
  lz_inds = traj_opt.mode_opt{i}.l_inds(3:3:end,:) + traj_opt.var_offset(i);
  force_cost = FunctionHandleObjective(numel(lz_inds),@force_cost_fun);
  traj_opt = traj_opt.addCost(force_cost,lz_inds(:));
end
% traj_opt = traj_opt.addModeRunningCost(2,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(3,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(4,@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);

for i=1:length(N)
  knee_inds = traj_opt.mode_opt{i}.x_inds([10;16],:) + traj_opt.var_offset(i);
  knee_inds = knee_inds(:);
  knee_constraint = BoundingBoxConstraint(.15*ones(length(knee_inds),1),inf(length(knee_inds),1));
  traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);
  
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:p.getNumPositions);
  
  % bound joint velocities
  joint_vel_max = 5; % was 10
  joint_vel_bound = BoundingBoxConstraint(-joint_vel_max*ones(p.getNumVelocities,1),joint_vel_max*ones(p.getNumVelocities,1));
  traj_opt = traj_opt.addModeStateConstraint(i,joint_vel_bound,1:N(i),p.getNumPositions+1:nx);
  
  
  % bound out of plane positions
  outplane_max = .2;
  outplane_inds = find(cellfun(@any,(regexp(p.getStateFrame.frame{1}.getCoordinateNames,'(base_y|_..x|_..z|roll|yaw)$'))));
  outplane_bound = BoundingBoxConstraint(-outplane_max*ones(length(outplane_inds),1),outplane_max*ones(length(outplane_inds),1));
  traj_opt = traj_opt.addModeStateConstraint(i,outplane_bound,1:N(i),outplane_inds);
end

traj_opt = traj_opt.compile();
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',traj_opt.num_vars+1);
if nargin == 2
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z0);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end

  function [f,df] = force_cost_fun(lz)
    K = 0.0001;
    f = K*(lz'*lz);
    df = 2*K*lz';
  end


  function [f,df] = running_cost_fun(h,x,u)
    R_diag = .1*ones(12,1);
    %     R_diag([1;3;5;6;7;8;11;12;13;14]) = 10*ones(10,1); %aky,akx
    R_diag([2;4;6;8;10;12]) = 10*ones(6,1);
    R = diag(R_diag);
%       = 0;
    %Also penalize out of plane joints
    Q_diag = zeros(nx,1);
    Q_diag(outplane_inds) = 10;
    Q_diag(nq+1:end) = 00;
    Q_diag(outplane_inds + nq) = 10;
    Q = diag(Q_diag);
%     Q = 0;
    
    f = h*u'*R*u + h*x'*Q*x;
    df = [u'*R*u+x'*Q*x 2*h*x'*Q 2*h*u'*R];
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    nu = length(u);  
    idx = [2;3;4;5;6];
    dot_idx = p.getNumPositions+idx;

    Kq = 5*0;
    Kqd = 50*0;
    f = Kq*x(idx)'*x(idx) + Kqd*x(dot_idx)'*x(dot_idx); % try to avoid moving the pelvis quickly
    df = zeros(1,1+nx+nu);
    df(1+idx) = 2*Kq*x(idx)';
    df(1+dot_idx) = 2*Kqd*x(dot_idx)';
    
    df = df*h;
    df(1) = f;
    f = f*h;
    
%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:nq);
    
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',true));
    phi0 = .2*ones(8,1);
    K = 1000;
    
%     [~,I1] = min(phi(1:2));
%     [~,I2] = min(phi(3:4));
%     phi = [phi(I1);phi(2+I2)];
%     n = [n(I1,:);n(2+I2,:)];    
%     phi0 = [.2;.2];
    
    I = find(phi < phi0);
    f = K*(phi(I) - phi0(I))'*(phi(I) - phi0(I));
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi(I)-phi0(I))'*n(I,:) zeros(1,nv+nu)];
    
    df = df*h;
    df(1) = f;
    f = f*h;
    
%     K = 500;
%     f = K*sum(phi0(I) - phi(I));
%     df = [0 K*sum(n(I,:),1) zeros(1,nv+nu)];

%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

end