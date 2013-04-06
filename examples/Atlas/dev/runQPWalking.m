function runQPWalking(num_steps,step_length,step_time)

if (nargin<1) num_steps = 10; end
if (nargin<2) step_length = 0.6; end
if (nargin<3) step_time = 0.8; end

options.floating = true;
options.dt = 0.003;
addpath(fullfile(getDrakePath,'examples','Atlas'));
addpath(fullfile(getDrakePath,'examples','ZMP'));
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);
v = r.constructVisualizer;
v.display_dt = 0.05;

% set initial state to fixed point
load('../data/atlas_fp.mat');
r = r.setInitialState(xstar);

nq = getNumDOF(r);
nu = getNumInputs(r);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

% create desired ZMP trajectory
[zmptraj,lfoottraj,rfoottraj,supptraj] = ZMPandFootTrajectory(r,q0,num_steps,step_length,step_time);
%[zmptraj,lfoottraj,rfoottraj,supptraj] = SteppingStonesTrajectory(r,q0,step_time);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

% construct ZMP feedback controller
com = getCOM(r,kinsol);
limp = LinearInvertedPendulum(com(3));
% get COM traj from desired ZMP traj
comtraj = ZMPplanner(limp,com(1:2),[0;0],zmptraj);

% time spacing of samples for IK
ts = 0:0.1:zmptraj.tspan(end);
T = ts(end);

% create desired joint trajectory
cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q0;
  
rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');

htraj = [];
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = inverseKin(r,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],rfoottraj.eval(t),lfoot_body,[0;0;0],lfoottraj.eval(t),options);
  else
    q = q0;
  end
  com = getCOM(r,q(:,i));
  htraj = [htraj com(3)];
  v.draw(t,q(:,i));
end
htraj = PPTrajectory(spline(ts,htraj));

figure(2); 
clf; 
subplot(3,1,1); hold on;
fnplt(zmptraj(1));
fnplt(comtraj(1));
subplot(3,1,2); hold on;
fnplt(zmptraj(2));
fnplt(comtraj(2));
subplot(3,1,3); hold on;
fnplt(htraj);

limp = LinearInvertedPendulum(htraj);
[~,V] = ZMPtracker(limp,zmptraj);

hddot = fnder(htraj,2);
zmpdata = SharedDataHandle(struct('S',V.S,'h',htraj,'hddot',hddot,'ti_flag',false));

% instantiate QP controller
options.exclude_torso = true;
options.slack_limit = 10.0;
options.w = 1.0;
options.R = 1e-12*eye(nu);
qp = QPController(r,zmpdata,options);

% desired configuration trajectory
qdes = setOutputFrame(PPTrajectory(spline(ts,q)),AtlasCoordinates(r));
pd = SimplePDController(r);
ins(1).system = 2;
ins(1).input = 2;
outs(1).system = 2;
outs(1).output = 1;
pd = mimoCascade(qdes,pd,[],ins,outs);
clear ins outs;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 1;
ins(2).input = 2;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins outs;

% walking foot support trajectory
ins(1).system = 2;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(supptraj,sys,[],ins,outs);
clear ins outs;

% feedback PD trajectory controller 
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],[],outs);
clear outs;

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],x0);
playback(v,traj,struct('slider',true));

for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumDOF(r)); 
  com(:,i)=getCOM(r,q);
end

figure(2);
subplot(3,1,1);
plot(ts,com(1,:),'r');
subplot(3,1,2);
plot(ts,com(2,:),'r');
subplot(3,1,3);
plot(ts,com(3,:),'r');

% keyboard;

end
