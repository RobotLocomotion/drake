function runWalkDemo(num_steps,step_length,step_time)
% NOTEST
if (nargin<1) num_steps = 8; end
if (nargin<2) step_length = 0.4; end
if (nargin<3) step_time = 1; end

sizecheck(num_steps,1);
rangecheck(num_steps,0,inf);
sizecheck(step_length,1);
sizecheck(step_time,1);

options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

if (0)  % add some stairs
  terrain = zeros(500,200);
  terrain(106:110,:)=1;
  terrain(111:115,:)=2;
  terrain(116:120,:)=3;
  terrain(121:125,:)=4;
  terrain(126:140,:)=5;
  terrain(141:145,:)=4;
  terrain(146:150,:)=3;
  terrain(151:155,:)=2;
  terrain(156:160,:)=1;
  r = setTerrain(r,terrain,[eye(3),[-100;100;0];0 0 0 10]);
end

% set initial state to fixed point
load('data/atlas_fp.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer();
v.display_dt = .05;
v.draw(0,xstar);

if (1)
  [Kp,Kd] = getPDGains(r);
  sys = pdcontrol(r,Kp,Kd);
  poscontrolsys = sys;
else
  r = enableIdealizedPositionControl(r,true);
  r = compile(r);
  sys = r;
end

%% produce basic stepping plan for zmp and feet

q0 = xstar(1:getNumDOF(r));
[zmptraj,lfoottraj,rfoottraj] = ZMPandFootTrajectory(r,q0,num_steps,step_length,step_time);


%% covert ZMP plan into COM plan using LIMP model
addpath(fullfile(getDrakePath,'examples','ZMP'));
[com,Jcom] = getCOM(r,q0);
comdot = Jcom*xstar(getNumDOF(r)+(1:getNumDOF(r)));
limp = LinearInvertedPendulum(com(3,1));

comtraj = [ ZMPplanner(limp,com(1:2),comdot(1:2),setOutputFrame(zmptraj,desiredZMP)); ...
  ConstantTrajectory(com(3,1)) ];
  
figure(2); 
clf; 
subplot(2,1,1); hold on;
fnplt(zmptraj(1));
fnplt(comtraj(1));
subplot(2,1,2); hold on;
fnplt(zmptraj(2));
fnplt(comtraj(2));

%% compute joint positions with inverse kinematics

ind = getActuatedJoints(r);
rfoot_body = r.findLinkInd('r_foot');
lfoot_body = r.findLinkInd('l_foot');

cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_bky = 100;
cost.back_bkx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q0;


ts = 0:.05:comtraj.tspan(end);
%ts = 0:.2:comtraj.tspan(end);

disp('computing ik...')
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = inverseKin(r,q(:,i-1),0,comtraj.eval(t),rfoot_body,zeros(3,1),rfoottraj.eval(t),lfoot_body,zeros(3,1),lfoottraj.eval(t),options);
  else
    q = q0;
  end
  q_d(:,i) = q(ind,i);
  v.draw(t,q(:,i));
end
q_dtraj = setOutputFrame(PPTrajectory(spline(ts,q_d)),getInputFrame(sys));

if (0)
  %% to view the motion plan:
  xtraj = setOutputFrame(PPTrajectory(spline(ts,[q;0*q])),getOutputFrame(sys));
  playback(v,xtraj,struct('slider',true));
  return;
end

sys = cascade(q_dtraj,sys);

disp('simulating system...');
T = ts(end); % sec
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T]);
save walking.mat;
playback(v,traj,struct('slider',true));


for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumDOF(r)); qd=x(getNumDOF(r)+1:end);
  com(:,i)=getCOM(r,q);
end

figure(2);
subplot(2,1,1);
plot(ts,com(1,:),'r');
subplot(2,1,2);
plot(ts,com(2,:),'r');


