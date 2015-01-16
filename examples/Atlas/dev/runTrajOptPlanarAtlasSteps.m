function runTrajOptPlanarAtlasSteps(passive_ankle)

if nargin<1
  passive_ankle = true;
end

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
l = 0.4;
h = 0.11;
boxes = [0.25+l, 0.0, 2*l, 1, h;
         0.25+l+l/2, 0.0, l, 1, 2*h];
options.terrain = RigidBodyStepTerrain(boxes);
% options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
if passive_ankle
  p = PlanarRigidBodyManipulator('../urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);
else
  p = PlanarRigidBodyManipulator('../urdf/atlas_simple_planar_contact.urdf',options);
end

% load precomputed kinematic seed
dd=load('data/atlas_step_qtraj.mat');

N = 30;
N1 = floor(N/2);
N2 = N-N1;
d = floor(N/8);

tf0 = dd.qtraj.tspan(2);
t_init = linspace(0,tf0,N);
q_vec = dd.qtraj.eval(t_init);
x_vec = [q_vec; 0*q_vec];
xtraj = PPTrajectory(foh(t_init,x_vec));
utraj = PPTrajectory(foh(t_init,0*randn(p.getNumInputs(),N)));

lp = [1;0;0;0];
ln = zeros(4,1);
ltraj = PPTrajectory(foh(t_init,[repmat([lp;lp;ln;ln],1,N1-d) zeros(16,2*d) repmat([ln;ln;lp;lp],1,N2-d)]));
ljltraj = [];

options.x0 = x_vec(:,1);
options.xf = x_vec(:,N);
options.T_span = [0.9*tf0, 1.1*tf0];
options.N = N;

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
  save('data/atlas_passive_ankle_steps_traj.mat','xtraj','utraj','ltraj','ljltraj','z','F');
else
  save('data/atlas_steps_traj.mat','xtraj','utraj','ltraj','ljltraj','z','F');
end
