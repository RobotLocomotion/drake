% megaclear
% traj_file = 'data/atlas_passive_ankle_lqr.mat';
% traj_file = 'data/atlas_passive_ankle_traj.mat';
% traj_file = 'data/atlas_passive_ankle_traj_fm2.mat';
passive_ankle = false;
traj_file = 'data/atlas_traj_fm';
load(traj_file);

%% get mode sequence
t = xtraj.pp.breaks;
l = ltraj.eval(t);
lz = l(1:4:end,:);
modes = lz > 1e-4;

N_vec = 1;
mode_seq_vec = modes(:,1);

for i=2:size(modes,2)-1,
  if isequal(modes(:,i),modes(:,i-1)),
    N_vec(end) = N_vec(end) + 1;
  else
    mode_seq_vec = [mode_seq_vec modes(:,i)];
    N_vec = [N_vec 1];
  end
end

mode_seq_vec
% keyboard

%% expand
N_vec_pre= N_vec;
N_vec = floor(2*N_vec);
t_pre = xtraj.pp.breaks;
t_init = t_pre(1);
sum(N_vec)
% keyboard
% 
for i=1:length(N_vec),
  t_span_mode = [t_pre(sum(N_vec_pre(1:i-1))+1) t_pre(sum(N_vec_pre(1:i))+1)];
  t_mode = linspace(t_span_mode(1),t_span_mode(2),N_vec(i)+1);
  t_init = [t_init t_mode(2:end)];
end

options.t_init = t_init;
options.N_vec = N_vec;
options.mode_seq_vec = mode_seq_vec;

%%


% load model
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
if passive_ankle
  p = PlanarRigidBodyManipulator('../urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);
else
  p = PlanarRigidBodyManipulator('../urdf/atlas_simple_planar_contact.urdf',options);
end

% build periodic constraint matrix
R_periodic = zeros(p.getNumStates,2*p.getNumStates);
R_periodic(2,2) = 1; %z
R_periodic(3,3) = 1; %pitch
R_periodic(4:6,8:10) = eye(3); %leg joints w/symmetry
R_periodic(8:10,4:6) = eye(3); %leg joints w/symmetry
R_periodic(7,7) = 1; % back joint
R_periodic(11:13,11:13) = eye(3); %x,z,pitch velocities
R_periodic(14:16,18:20) = eye(3); %leg joints w/symmetry
R_periodic(18:20,14:16) = eye(3); %leg joints w/symmetry
R_periodic(17,17) = 1; % back joint
R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);
options.R_periodic = R_periodic;

options.T_span = [.375 .75];
% N = 40;
% N1 = floor(N/2);
% N2 = N-N1;
% d = floor(N/8);
% tf0 = 0.75;

% n_knots = 5;
% ts = linspace(0,tf0,n_knots);
% nq = p.getNumPositions;
%
% qknots = zeros(nq,n_knots);
% qknots(:,1) = [0;0;.2;-.3;.2;-0.1;0;-.5;1.1;0];
% phi = p.contactConstraints(qknots(:,1));
% z = -min(phi);
% qknots(2,1) = z;
%
% qknots(:,2) = [0;z;.2;-.3;.2;-0.1;0;-0.9;1.3;0];
%
% qknots(:,3) = [0.15;0;.3;-.1;.2;-0.1;0;-0.9;0.3;0];
% phi = p.contactConstraints(qknots(:,3));
% z = -min(phi);
% qknots(2,3) = z;
%
% qknots(:,4) = [0.15;0;.4;-.1;.2;-0.1;0;-0.9;0.3;0];
% phi = p.contactConstraints(qknots(:,4));
% z = -min(phi);
% qknots(2,4) = z;
%
% qknots(:,5) = [0.3;0;.2;-.5;1.1;0;0;-.3;.2;-0.1];
% phi = p.contactConstraints(qknots(:,5));
% z = -min(phi);
% qknots(2,5) = z;
%
% qtraj = PPTrajectory(foh(ts,qknots));
%
% options.T_span = [tf0/2, tf0];
% options.N = N;
%
% t_init = linspace(0,tf0,N);
% q_vec = qtraj.eval(t_init);
% x_vec = [q_vec;0*q_vec];
% x_vec(nq+1,:) = 0.4; % reasonable foward velocity
%
% options.x0 = x_vec(:,1);
% options.xf = x_vec(:,end);
%
% xtraj = PPTrajectory(foh(t_init,x_vec));
% utraj = PPTrajectory(foh(t_init,0*randn(p.getNumInputs,N)));
% lp = [1;0;0;0];
% ln = zeros(4,1);
% ltraj = PPTrajectory(foh(t_init,[repmat([lp;lp;ln;ln],1,N1-d) zeros(16,2*d) repmat([ln;ln;lp;lp],1,N2-d)]));
% ljltraj = [];

if 1
  v = p.constructVisualizer;
  [xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = fixedModeTrajOptPlanarAtlas(p,xtraj,utraj,ltraj,[],options);
  v.playback(xtraj);
else
  v = p.constructVisualizer;
  options.N = sum(options.N_vec)+1;
  options.scale = 0;
  [xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = trajOptPlanarAtlas(p,xtraj,utraj,ltraj,[],options);
  v.playback(xtraj);
end

v.playback(xtraj,struct('slider',true));