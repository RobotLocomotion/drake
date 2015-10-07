function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = test3DSingleModeDircol(x0,xf,N,duration,mode,active_inds,z0)

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

%  x0 = init3DTraj.threeDTraj.xtraj{1}.eval(0);

to_options = struct();
to_options.active_inds = active_inds;

traj_opt = ContactConstrainedDircolTrajectoryOptimization(p,N,duration,mode,to_options);

l0 = [0;0;897.3515;0;0;897.3515;0;0;179.1489;0;0;179.1489]/2;

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0,x0),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf,xf),N);

t_init = linspace(0,duration(2),N);
traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
traj_init.u = PPTrajectory(foh(t_init,randn(nu,N(1))));
% traj_init.l = PPTrajectory(foh(t_init,repmat(l0,1,N(1))));

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt2.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',2000000);



% traj_opt = traj_opt.addModeRunningCost(1,@foot_height_fun);
% traj_opt = traj_opt.setCheckGrad(true);
% for i=1:length(N)
  traj_opt = traj_opt.addRunningCost(@running_cost_fun);
  traj_opt = traj_opt.addRunningCost(@pelvis_motion_fun);
%   
% end

for i=1:length(N)
%   knee_inds = traj_opt.mode_opt{i}.x_inds([5;9],:) + traj_opt.var_offset(i);
%   knee_inds = knee_inds(:);
%   knee_constraint = BoundingBoxConstraint(.1*ones(length(knee_inds),1),inf(length(knee_inds),1));
%   traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);
  
  traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:p.getNumPositions);
  
  % bound joint velocities
  joint_vel_max = 10;
  joint_vel_bound = BoundingBoxConstraint(-joint_vel_max*ones(p.getNumVelocities,1),joint_vel_max*ones(p.getNumVelocities,1));
  traj_opt = traj_opt.addStateConstraint(joint_vel_bound,1:N(i),p.getNumPositions+1:nx);
  
  
  % bound out of plane positions
  outplane_max = .2;
  outplane_inds = find(cellfun(@any,(regexp(p.getStateFrame.coordinates(1:21),'(base_y|_..x|_..z|roll|yaw)$'))));
  outplane_bound = BoundingBoxConstraint(-outplane_max*ones(length(outplane_inds),1),outplane_max*ones(length(outplane_inds),1));
  traj_opt = traj_opt.addStateConstraint(outplane_bound,1:N(i),outplane_inds);
end

% traj_opt = traj_opt.compile();
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',traj_opt.num_vars+1);
if nargin == 7
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z0);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end


  function [f,df] = running_cost_fun(h,x,u)
    R_diag = 1*ones(12,1);
    %     R_diag([1;3;5;6;7;8;11;12;13;14]) = 10*ones(10,1); %aky,akx
%     R_diag([1;3;5;7;9;11;13;15]) = 10*ones(8,1);
    R_diag([2;4;6;8;10;12]) = 20*ones(6,1);
    R = diag(R_diag);
    
    %Also penalize out of plane joints
    Q_diag = zeros(nx,1);
    Q_diag(nq+1:end) = 00;
    Q_diag(outplane_inds + nq) = 100;
    Q = diag(Q_diag);
%     Q = 0;
    
    f = h*u'*R*u + h*x'*Q*x;
    df = [u'*R*u+x'*Q*x 2*h*x'*Q 2*h*u'*R];
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    nu = length(u);  
    idx = [2;3;4;5;6];
    dot_idx = p.getNumPositions+idx;

    Kq = 5;
    Kqd = 50;
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
    K = 5000;
    
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