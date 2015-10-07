function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = modes1and2StepDircol(z0,xtraj,utraj,ltraj)
step_height = .3;

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
% options.terrain = RigidBodyFlatTerrain();
options.terrain = RigidBodyLinearStepTerrain(step_height,.35,.02);
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;

p = PlanarRigidBodyManipulator('../urdf/atlas_planar_one_arm_noback.urdf',options);
nq = p.getNumPositions;
nv = p.getNumVelocities;
nu = p.getNumInputs;
nx = p.getNumStates();
v = p.constructVisualizer(struct('viewer','BotVisualizer'));

% N = [8,8,5];
% % N = [2,2,2,2,2];
% % N = N+1;
% % N = [7,3,3,3,7];
% N = [6,6,5,5,6];
N = [12,8,1];
duration = {[.2 1.5],[.1 1.5],[0 0]};
modes = {[1;2],[1;2;5],[2;3;4;5]};

% N = N(1:4);
% duration = duration(1:4);
% modes = modes(1:4);


x0 = [      0
    0.9371
    0.2000
   -0.4414
    0.2625
   -0.0211
   0   
   -0.4997
    0.9273
   -0.6403
   0   
    0.2346
   -0.0077
    0.0731
   -0.2012
    0.7876
   -0.6596
   0       
   -1.9375
    2.5602
   -2.2319
   0   ]; 
 
 % position with hand near ground, both feet on ground
qf=[    0.1631
    0.9309
    1.2733
   -1.4564
    0.4207
   -0.2375
   -0.9852
   -2.8021
    1.5860
   -0.0571
   -0.2447];
 
 [~,normal,d,xA,xB,idxA,idxB,mu,n,D] = contactConstraints(p,x0(1:p.getNumPositions));
 assert(isequal(idxB,ones(size(idxB))))
 
 
% to_options.u_const_across_transitions = true;
% to_options.lambda_bound = 20; %was 400

to_options.mode_options{1}.active_inds = [1;2;4];
to_options.mode_options{2}.active_inds = [1;2;4;5;6];
to_options.mode_options{3}.active_inds = [1;2;3;4;6;7;8];

% 
% to_options.mode_options{1}.non_penetration = false;
% to_options.mode_options{2}.non_penetration = false;

contact_q0 = zeros(11,1);
contact_q0(4) = -.5;  %right hip
contact_q0(8) = -1.5;  %right hip
contact_q0(3) = .5; %pitch
contact_q0(7) = -1.5; % shoulder

contact_q0_final = contact_q0;
contact_q0_final(4) = -1.5;

to_options.mode_options{1}.contact_q0 =  contact_q0;
to_options.mode_options{2}.contact_q0 =  contact_q0;
to_options.mode_options{3}.contact_q0 =  contact_q0;

traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);

% l0 = [0;897.3515;0;179.1489];
l0 = [0;400];


% Add foot height constraint
fnx = drakeFunction.kinematic.WorldPosition(p,idxA(4),p.T_2D_to_3D'*xA(:,4),1);
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(-inf,.3,p.getNumPositions, @fnx.eval, 1),N(1), 1:p.getNumPositions);

fn1_all = drakeFunction.kinematic.WorldPosition(p,idxA(3),p.T_2D_to_3D'*xA(:,3));
fn2_all = drakeFunction.kinematic.WorldPosition(p,idxA(4),p.T_2D_to_3D'*xA(:,4));
fn1 = drakeFunction.kinematic.WorldPosition(p,idxA(3),p.T_2D_to_3D'*xA(:,3),2);
fn2 = drakeFunction.kinematic.WorldPosition(p,idxA(4),p.T_2D_to_3D'*xA(:,4),2);

% for j=1:,
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint([-inf;.1;],[.3;inf],p.getNumPositions, @fn2_all.eval, 1), 2:N(1), 1:p.getNumPositions);
traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.2,inf,p.getNumPositions, @fn1.eval, 1), 2:N(1), 1:p.getNumPositions);
% end
% traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.2,inf,p.getNumPositions, @fn1.eval, 1), floor(N(1)/2), 1:p.getNumPositions);
% traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.2,inf,p.getNumPositions, @fn2.eval, 1), floor(N(1)/2), 1:p.getNumPositions);

traj_opt = traj_opt.addModeStateConstraint(2,FunctionHandleConstraint(.37,inf,p.getNumPositions, @fn1.eval, 1), N(2)-2, 1:p.getNumPositions);
traj_opt = traj_opt.addModeStateConstraint(2,FunctionHandleConstraint(.37,inf,p.getNumPositions, @fn2.eval, 1), N(2)-2, 1:p.getNumPositions);

% traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;.1*ones(13,1);.1*ones(14,1)],x0+[0;.1*ones(13,1);.1*ones(14,1)]),1);
traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;.3*ones(nq-1,1);3*ones(nq,1)],x0+[0;.3*ones(nq-1,1);3*ones(nq,1)]),1);
inds = [1:6 8:10];
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(qf(inds) - .02*ones(nq-2,1),qf(inds) + .02*ones(nq-2,1)),N(end),inds);

% to push the foot and hand over the lip
if length(N) > 1
  traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(-.1,inf),traj_opt.mode_opt{2}.cstrval_inds(2) + traj_opt.var_offset(2));
end

for i=1:length(N)
  t_init{i} = linspace(0,mean(duration{i}),N(i));
  if i >3
    traj_init.mode{i}.x = PPTrajectory(foh(t_init{i},repmat([q_mid;q_mid*0],1,N(i))));
  else
    traj_init.mode{i}.x = PPTrajectory(foh(t_init{i},repmat(x0,1,N(i))));
  end
  traj_init.mode{i}.u = PPTrajectory(foh(t_init{i},randn(nu,N(i))));
  traj_init.mode{i}.l = PPTrajectory(foh(t_init{i},repmat(l0,length(modes{i}),N(i))));
end

if nargin > 1
  for i=1:length(N)
    t_init{i} = linspace(0,diff(xtraj{i}.tspan),N(i));
    traj_init.mode{i}.x = xtraj{i};
    if nargin < 2
      traj_init.mode{i}.u = utraj{i};
    end
    if nargin > 3
      traj_init.mode{i}.l = ltraj{i};
    end
  end
end

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt2.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',100);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);


%   traj_opt = traj_opt.addModeRunningCost(1,@foot_height_fun);
% traj_opt = traj_opt.setCheckGrad(true);
for i=1:length(N)
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@pelvis_motion_fun);
end
% traj_opt = traj_opt.addModeRunningCost(2,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(3,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(4,@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);

for i=1:length(N)
  knee_inds = traj_opt.mode_opt{i}.x_inds([5;9],:) + traj_opt.var_offset(i);
  knee_inds = knee_inds(:);
  knee_constraint = BoundingBoxConstraint(.1*ones(length(knee_inds),1),inf(length(knee_inds),1));
  traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);
  
%   traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:14);


  el_ind = 11;
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(.1,inf),1:N(i),el_ind);
  
  aky_ind = 10;
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(-1,inf),1:N(i),aky_ind);
  
  % bound joint velocities
  joint_vel_max = 10;
  joint_vel_bound = BoundingBoxConstraint(-joint_vel_max*ones(p.getNumVelocities,1),joint_vel_max*ones(p.getNumVelocities,1));
  traj_opt = traj_opt.addModeStateConstraint(i,joint_vel_bound,1:N(i),nq+1:nx);
end

traj_opt = traj_opt.compile();
if nargin == 1
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z0);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end


  function [f,df] = running_cost_fun(h,x,u)
    Q_diag = ones(nx,1);
    Q_diag(nq+1:end) = ones(nv,1);
    Q = diag(Q_diag);
    
    R = .1;
    f = h*u'*R*u + h*x'*Q*x;
    df = [u'*R*u+x'*Q*x 2*h*x'*Q 2*h*u'*R];    
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    nu = length(u);  
    pitch_idx = 3;
    pitch_dot_idx = p.getNumPositions+pitch_idx;

    Kq = 0;
    Kqd = 0;
    f = Kq*x(pitch_idx)^2 + Kqd*x(pitch_dot_idx)^2; % try to avoid moving the pelvis quickly
    df = zeros(1,1+nx+nu);
    df(1+pitch_idx) = 2*Kq*x(pitch_idx);
    df(1+pitch_dot_idx) = 2*Kqd*x(pitch_dot_idx);
    
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
    phi0 = [.3;.3;.3;.3];
    K = 5;
    
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