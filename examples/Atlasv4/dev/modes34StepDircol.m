function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = modes34StepDircol(z0,xtraj,utraj,ltraj)
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
N = [5,10,5];
duration = {[.02 .2] [.5 1.5] [.05 1.5]};
modes = {[1;2;3;4;5], [3;4], [1;2;3;4]};

N = N(1:2);
duration = duration(1:2);
modes = modes(1:2);


x0 = [    0.1831
    0.9319
    1.2869
   -1.4764
    0.4070
   -0.2175
   -1.5300
   -2.8093
    1.5995
   -0.0771
    0.1000
   -0.0589
   -0.1848
   -0.3119
   -1.6396
    3.6395
   -1.6880
   -0.4325
   -0.2153
    0.5184
    0.0088
    1.0700]; 
  
    % from clearance traj
  x0=[    0.1831
    0.9319
    1.2869
   -1.4764
    0.4070
   -0.2175
   -1.5300
   -2.8093
    1.5995
   -0.0771
    0.1000
   -0.0648
   -0.1958
   -0.3235
   -1.7483
    3.8575
   -1.7856
   -0.5391
   -0.2351
    0.5427
    0.0159
    1.2556
];
 
 
 [~,normal,d,xA,xB,idxA,idxB,mu,n,D] = contactConstraints(p,x0(1:p.getNumPositions));
 assert(isequal(idxB,ones(size(idxB))))
 
 
% to_options.u_const_across_transitions = true;
% to_options.lambda_bound = 20; %was 400

% to_options.mode_options{1}.active_inds = [1;2;3;4;6;7;8];
% to_options.mode_options{2}.active_inds = [1;2;4;5;6];
to_options.mode_options{1}.active_inds = [1;2;4;5;6;8;9;10];
to_options.mode_options{2}.active_inds = [1;2;4];
to_options.mode_options{3}.active_inds = [1;2;4;5;6;8];
% 
% to_options.mode_options{1}.non_penetration = false;
% to_options.mode_options{2}.non_penetration = false;
% to_options.mode_options{3}.non_penetration = false;
% to_options.mode_options{4}.non_penetration = false;
% to_options.mode_options{4}.non_penetration = false;

contact_q0 = zeros(11,1);
contact_q0(4) = -.5;  %right hip
contact_q0(8) = -1.5;  %right hip
contact_q0(3) = .5; %pitch
contact_q0(7) = -1.5; % shoulder

contact_q0_final = contact_q0;
contact_q0_final(4) = -1.5;

to_options.mode_options{1}.contact_q0 =  contact_q0;
to_options.mode_options{2}.contact_q0 =  contact_q0;
to_options.mode_options{3}.contact_q0 =  contact_q0_final;

traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);

% l0 = [0;897.3515;0;179.1489];
l0 = [0;400];


% Add foot height constraint
% fn1_all = drakeFunction.kinematic.WorldPosition(p,idxA(3),p.T_2D_to_3D'*xA(:,3));
fn1_all = drakeFunction.kinematic.WorldPosition(p,idxA(1),p.T_2D_to_3D'*xA(:,1));
fn1 = drakeFunction.kinematic.WorldPosition(p,idxA(1),p.T_2D_to_3D'*xA(:,1),2);
fn2 = drakeFunction.kinematic.WorldPosition(p,idxA(2),p.T_2D_to_3D'*xA(:,2),2);

traj_opt = traj_opt.addModeStateConstraint(2,FunctionHandleConstraint(.04,inf,p.getNumPositions, @fn1.eval, 1), 2, 1:p.getNumPositions);
traj_opt = traj_opt.addModeStateConstraint(2,FunctionHandleConstraint(.04,inf,p.getNumPositions, @fn2.eval, 1), 2, 1:p.getNumPositions);


% traj_opt = traj_opt.addModeStateConstraint(2,FunctionHandleConstraint([-inf;.4;],[.3;inf],p.getNumPositions, @fn1_all.eval, 1), floor(N(2)/2), 1:p.getNumPositions);

% traj_opt = traj_opt.addModeStateConstraint(2,FunctionHandleConstraint(.35,inf,p.getNumPositions, @fn1.eval, 1), N(2)-2, 1:p.getNumPositions);
% traj_opt = traj_opt.addModeStateConstraint(2,FunctionHandleConstraint(.35,inf,p.getNumPositions, @fn2.eval, 1), N(2)-2, 1:p.getNumPositions);

% traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;.1*ones(13,1);.1*ones(14,1)],x0+[0;.1*ones(13,1);.1*ones(14,1)]),1);
traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0-.001,x0+.001),1);



if length(N) >= 2
  qf = [         0
    2.0000
    0.1000
   -0.2000
    0.2000
   -0.1000
         0
   -0.2000
    0.2000
   -0.1000
   -0.5000];
 
 qf = [
         0
    0.9460
    0.5000
   -0.7997
    0.9925
   -0.5412
   -0.3000
   -0.6900
    0.1000
    0.0900
    0.1000
];
 qf_ind = [3:11];
  traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(qf(qf_ind),qf(qf_ind)),N(end),qf_ind);
%   traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(.4,inf),N(end),1);
end

%   traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(-inf,.15),traj_opt.mode_opt{1}.cstrval_inds(1) + traj_opt.var_offset(1));

% to push the foot and hand over the lip
if length(N) > 1
%   traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(-.33,inf),traj_opt.mode_opt{2}.cstrval_inds(2) + traj_opt.var_offset(2));
end

if length(N) > 2
%   traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(-.33,inf),traj_opt.mode_opt{3}.cstrval_inds(2) + traj_opt.var_offset(3));
end
if length(N) == 2
%   traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(-.3,inf),traj_opt.mode_opt{5}.cstrval_inds(1) + traj_opt.var_offset(5));
end


for i=1:length(N)
  t_init{i} = linspace(0,mean(duration{i}),N(i));
  traj_init.mode{i}.x = PPTrajectory(foh(t_init{i},repmat(x0,1,N(i))));
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

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
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
%   traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);
  
%   traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:14);

  aky_ind = 6;
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(-1,inf),1:N(i),aky_ind);
  
    el_ind = 11;
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(.1,inf),1:N(i),el_ind);
  
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
    Q_diag(nq+1:end) = 5*ones(nv,1);
    Q = diag(Q_diag);
    
    R = .1;
    f = h*u'*R*u + h*x'*Q*x;
    df = [u'*R*u+x'*Q*x 2*h*x'*Q 2*h*u'*R];    
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    nu = length(u);  
    pitch_idx = 3;
    pitch_dot_idx = p.getNumPositions+pitch_idx;

    Kq = 50;
    Kqd = 50;
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