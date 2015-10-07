function runTrajOptPlanarAtlas(passive_ankle)

if nargin<1
  passive_ankle = true;
end

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

N = 30;
N1 = floor(N/2);
N2 = N-N1;
d = floor(N/8);
tf0 = 0.75;

n_knots = 5;
ts = linspace(0,tf0,n_knots);
nq = p.getNumPositions;

qknots = zeros(nq,n_knots);
qknots(:,1) = [0;0;.2;-.3;.2;-0.1;0;-.5;1.1;0];
phi = p.contactConstraints(qknots(:,1));
z = -min(phi);
qknots(2,1) = z;

qknots(:,2) = [0;z;.2;-.3;.2;-0.1;0;-0.9;1.3;0];

qknots(:,3) = [0.15;0;.3;-.1;.2;-0.1;0;-0.9;0.3;0];
phi = p.contactConstraints(qknots(:,3));
z = -min(phi);
qknots(2,3) = z;

qknots(:,4) = [0.15;0;.4;-.1;.2;-0.1;0;-0.9;0.3;0];
phi = p.contactConstraints(qknots(:,4));
z = -min(phi);
qknots(2,4) = z;

qknots(:,5) = [0.3;0;.2;-.5;1.1;0;0;-.3;.2;-0.1];
phi = p.contactConstraints(qknots(:,5));
z = -min(phi);
qknots(2,5) = z;

qtraj = PPTrajectory(foh(ts,qknots));

% xvel = 0.5;
% q0 = [0;0;.2;-.4;.2;0;0;-1;1.5;0];
% phi_tmp = p.contactConstraints(q0);
% q0(2) = -phi_tmp(1);
% x0 = [q0;0.5;zeros(9,1)];
% xf = -pinv(R_periodic(:,21:end))*(R_periodic(:,1:20)*x0);
% xf(1) = .4;

% % guess an intermediary state
% q1 = [.20;0;0;.1;.2;-.3;0;-1;.5;0];
% phi_tmp = p.contactConstraints(q1);
% q1(2) = -phi_tmp(1);
% q2 = [xf(1);0;.2;.5;.4;0;0;-.4;.2;0];
% phi_tmp = p.contactConstraints(q2);
% q2(2) = -phi_tmp(3);

% x1 = [q1;xvel;zeros(9,1)];
% x2 = [q2;xvel;zeros(9,1)];

options.T_span = [tf0/2, tf0];
options.N = N;

t_init = linspace(0,tf0,N);
q_vec = qtraj.eval(t_init);
x_vec = [q_vec;0*q_vec];
x_vec(nq+1,:) = 0.4; % reasonable foward velocity

options.x0 = x_vec(:,1);
options.xf = x_vec(:,end);

xtraj = PPTrajectory(foh(t_init,x_vec));
utraj = PPTrajectory(foh(t_init,0*randn(p.getNumInputs,N)));
lp = [1;0;0;0];
ln = zeros(4,1);
ltraj = PPTrajectory(foh(t_init,[repmat([lp;lp;ln;ln],1,N1-d) zeros(16,2*d) repmat([ln;ln;lp;lp],1,N2-d)]));
ljltraj = [];

v = p.constructVisualizer; 
scales = [1,0.1,0.01,0.001,0];
for i=1:length(scales)
  options.scale = scales(i);
  [xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = trajOptPlanarAtlas(p,xtraj,utraj,ltraj,ljltraj,options);
  v.playback(xtraj);
  fprintf('iter %d\n',i)
end

v.playback(xtraj,struct('slider',true));
keyboard;
if passive_ankle
  save('data/atlas_passive_ankle_traj.mat','xtraj','utraj','ltraj','ljltraj','z','F');
else
  save('data/atlas_traj.mat','xtraj','utraj','ltraj','ljltraj','z','F');
end