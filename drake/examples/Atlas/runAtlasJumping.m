function runAtlasJumping(use_mex,use_angular_momentum)

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

if (nargin<1); use_mex = true; end
if (nargin<2); use_angular_momentum = false; end

import atlasControllers.*;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.001;
options.ignore_effort_limits = false;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

v = r.constructVisualizer;
v.display_dt = 0.005;

% load in running trajectory
sol = load([getDrakePath,'/solvers/trajectoryOptimization/dev/test_jump_v5.mat'],'xtraj_sol','com_sol','comdot_sol','comddot_sol','t_sol');

ts = unique(sol.xtraj_sol.getBreaks);
xtraj = sol.xtraj_sol;
r.setInitialState(xtraj.eval(0));
x_knots = xtraj.eval(ts);

back_bky = r.getBody(r.findJointId('back_bky')).position_num;

link_indices = [findLinkId(r,'l_foot'), findLinkId(r,'r_foot'),...
    findLinkId(r,'l_hand'), findLinkId(r,'r_hand'), ...
    findLinkId(r,'pelvis'), findLinkId(r,'utorso')];

body_knots = cell(length(link_indices));
for j=1:length(link_indices)
  body_knots{j} = zeros(6,length(ts));
end

nq = getNumPositions(r);
for i=1:length(ts)
  qi = x_knots(1:nq,i);
  kinsol = doKinematics(r,qi);
  for j=1:length(link_indices)
    body_knots{j}(:,i) = forwardKin(r,kinsol,link_indices(j),[0;0;0],1);
  end
end

for j=1:length(link_indices)
  link_con = struct();
  link_con.link_ndx = link_indices(j);
  link_con.pt = [0;0;0];
  link_con.traj = PPTrajectory(foh(ts,body_knots{j}));
  link_con.dtraj = fnder(link_con.traj);
  link_con.ddtraj = fnder(link_con.traj,2);
  link_constraints(j) = link_con;
end

com = sol.com_sol;
comdot = sol.comdot_sol;
comddot = sol.comddot_sol;
ts = sol.t_sol;

% for i=0:0.01:ts(end)
%   v.draw(i,xtraj.eval(i));
%   pause(0.01);
% end


% manually extracting the support traj for now
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');

toe_takeoff_idx = 7;
toe_land_idx = 12;
nT = 17;

flight = RigidBodySupportState(r,[]);
foot_support = RigidBodySupportState(r,[l_foot,r_foot]);
toe_support = RigidBodySupportState(r,[l_foot,r_foot],struct('contact_groups',{{{'toe'},{'toe'}}}));

supports = [foot_support;foot_support;foot_support;foot_support;foot_support;...
  toe_support;toe_support;...
  flight;flight;flight;flight;...
  toe_support;toe_support;
  foot_support;foot_support;foot_support;foot_support];
r = r.setInitialState(x_knots(:,1));

% build TV-LQR controller on COM dynamics
x0traj = PPTrajectory(foh(ts,[com;comdot]));
x0traj = x0traj.setOutputFrame(atlasFrames.COMState);
u0traj = PPTrajectory(foh(ts,comddot));
u0traj = u0traj.setOutputFrame(atlasFrames.COMAcceleration);

x0traj_landing = PPTrajectory(foh(ts(toe_land_idx:nT),[com(:,toe_land_idx:nT);comdot(:,toe_land_idx:nT)]));
x0traj_landing = x0traj_landing.setOutputFrame(atlasFrames.COMState);
u0traj_landing = PPTrajectory(foh(ts(toe_land_idx:nT),comddot(:,toe_land_idx:nT)));
u0traj_landing = u0traj_landing.setOutputFrame(atlasFrames.COMAcceleration);

x0traj_takeoff = PPTrajectory(foh(ts(1:toe_takeoff_idx),[com(:,1:toe_takeoff_idx);comdot(:,1:toe_takeoff_idx)]));
x0traj_takeoff = x0traj_takeoff.setOutputFrame(atlasFrames.COMState);
u0traj_takeoff = PPTrajectory(foh(ts(1:toe_takeoff_idx),comddot(:,1:toe_takeoff_idx)));
u0traj_takeoff = u0traj_takeoff.setOutputFrame(atlasFrames.COMAcceleration);

Q = 10*diag([10 10 10 1 1 1]);
R = 0.0001*eye(3);
A = [zeros(3),eye(3); zeros(3,6)];
B = [zeros(3); eye(3)];
options.tspan = ts;
options.sqrtmethod = false;
ti_sys = LinearSystem(A,B,[],[],eye(6),[]);
ti_sys = ti_sys.setStateFrame(atlasFrames.COMState);
ti_sys = ti_sys.setOutputFrame(atlasFrames.COMState);
ti_sys = ti_sys.setInputFrame(atlasFrames.COMAcceleration);
[~,V] = tvlqr(ti_sys,x0traj,u0traj,Q,R,Q,options);

[~,Vf] = tilqr(ti_sys,[com(:,end);zeros(3,1)],zeros(3,1),Q,R);
options_landing = options;
options_landing.tspan = [ts(toe_land_idx) ts(nT)];
[~,V_landing] = tvlqr(ti_sys,x0traj_landing,u0traj_landing,Q,R,Vf.S,options_landing);
A_flight = [eye(3) (ts(toe_land_idx)-ts(toe_takeoff_idx))*eye(3);zeros(3) eye(3)];
S_landing = V_landing.S.eval(V_landing.S.tspan(1));
s1_landing = V_landing.s1.eval(V_landing.s1.tspan(1));
s2_landing = V_landing.s2.eval(V_landing.s2.tspan(1));
S_takeoff = A_flight'*S_landing*A_flight;
s1_takeoff = A_flight'*s1_landing;
s2_takeoff = s2_landing;
options_takeoff = options;
options_takeoff.tspan = [ts(1) ts(toe_takeoff_idx)];
[~,V_takeoff] = tvlqr(ti_sys,x0traj_takeoff,u0traj_takeoff,Q,R,{S_takeoff;s1_takeoff;s2_takeoff},options_takeoff);
S_flight_values = zeros(6,6,2);
S_flight_values(:,:,1) = S_takeoff;
S_flight_values(:,:,2) = S_landing;
S_flight = PPTrajectory(foh([ts(toe_takeoff_idx) ts(toe_land_idx)],S_flight_values));
V_S = V_takeoff.S.append(S_flight);
V_S = V_S.append(V_landing.S);
s1_flight = PPTrajectory(foh([ts(toe_takeoff_idx) ts(toe_land_idx)],[s1_takeoff s1_landing]));
V_s1 = V_takeoff.s1.append(s1_flight);
V_s1 = V_s1.append(V_landing.s1);
s2_flight = PPTrajectory(foh([ts(toe_takeoff_idx) ts(toe_land_idx)],[s2_takeoff s2_landing]));
V_s2 = V_takeoff.s2.append(s2_flight);
V_s2 = V_s2.append(V_landing.s2);

ctrl_data = QPControllerData(true,struct(...
  'acceleration_input_frame',atlasFrames.AtlasCoordinates(r),...
  'A',A,...
  'B',B,...
  'C',eye(6),...
  'D',zeros(6,3),...
  'Qy',0*Q,...
  'R',R,...
  'S',V_S,...
  's1',V_s1,...
  's2',V_s2,...
  'x0',x0traj,...
  'u0',u0traj,...
  'y0',x0traj,...
  'qtraj',xtraj(1:nq),...
  'support_times',ts,...
  'supports',supports,...
  'mu',1.0,...
  'ignore_terrain',false,...
  'link_constraints',link_constraints,...
  'constrained_dofs',[]));

% instantiate QP controller
options.slack_limit = 1000;
options.w_qdd = 1e-5*ones(nq,1);
% options.w_qdd(back_bky) = 0.01;
options.w_qdd(1:6) = 0;
options.w_grf = 0;
options.w_slack = 3;
options.debug = false;
options.use_mex = use_mex;
options.contact_threshold = 0.0005;

if use_angular_momentum
  options.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  options.W_kdot = 1e-5*eye(3); % angular momentum weight
else
  options.W_kdot = zeros(3); 
end

boptions.Kp =250*ones(6,1);
boptions.Kd = 2*sqrt(boptions.Kp);
boptions_foot.Kp = 500*ones(6,1);
boptions_foot.Kd = 2*sqrt(boptions_foot.Kp);
lfoot_motion = BodyMotionControlBlock(r,'l_foot',ctrl_data,boptions_foot);
rfoot_motion = BodyMotionControlBlock(r,'r_foot',ctrl_data,boptions_foot);
pelvis_motion = BodyMotionControlBlock(r,'pelvis',ctrl_data,boptions);
lhand_motion = BodyMotionControlBlock(r,'l_hand',ctrl_data,boptions);
rhand_motion = BodyMotionControlBlock(r,'r_hand',ctrl_data,boptions);
% boptions.Kp(4:6) = NaN; % don't constrain orientation
% boptions.Kd(4:6) = NaN;
torso_motion = BodyMotionControlBlock(r,'utorso',ctrl_data,boptions);


motion_frames = {lfoot_motion.getOutputFrame,rfoot_motion.getOutputFrame,...
  lhand_motion.getOutputFrame,rhand_motion.getOutputFrame,...
	pelvis_motion.getOutputFrame,torso_motion.getOutputFrame};

options.body_accel_input_weights = 0*[100 100 10 10 100 10];
qp = QPController(r,motion_frames,ctrl_data,options);

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 2;
ins(2).system = 1;
ins(2).input = 3;
ins(3).system = 1;
ins(3).input = 4;
ins(4).system = 1;
ins(4).input = 5;
ins(5).system = 1;
ins(5).input = 6;
ins(6).system = 1;
ins(6).input = 7;
ins(7).system = 1;
ins(7).input = 8;
ins(8).system = 1;
ins(8).input = 9;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins;

% feedback foot contact detector with QP/atlas
options.use_lcm=false;
fc = FootContactBlock(r,ctrl_data,options);
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 3;
ins(3).system = 2;
ins(3).input = 4;
ins(4).system = 2;
ins(4).input = 5;
ins(5).system = 2;
ins(5).input = 6;
ins(6).system = 2;
ins(6).input = 7;
ins(7).system = 2;
ins(7).input = 8;
sys = mimoFeedback(fc,sys,[],[],ins,outs);
clear ins;  
  
% feedback PD block
% options.Kp =250*ones(34,1);
% options.Kd = 2*sqrt(options.Kp);
options.use_ik = false;
pd = IKPDBlock(r,ctrl_data,options);
ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
ins(3).system = 2;
ins(3).input = 3;
ins(4).system = 2;
ins(4).input = 4;
ins(5).system = 2;
ins(5).input = 5;
ins(6).system = 2;
ins(6).input = 6;
ins(7).system = 2;
ins(7).input = 7;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins;

% feedback body motion control blocks
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 3;
ins(3).system = 2;
ins(3).input = 4;
ins(4).system = 2;
ins(4).input = 5;
ins(5).system = 2;
ins(5).input = 6;
ins(6).system = 2;
ins(6).input = 7;
sys = mimoFeedback(lfoot_motion,sys,[],[],ins,outs);
clear ins;

ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 3;
ins(3).system = 2;
ins(3).input = 4;
ins(4).system = 2;
ins(4).input = 5;
ins(5).system = 2;
ins(5).input = 6;
sys = mimoFeedback(rfoot_motion,sys,[],[],ins,outs);
clear ins;

ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 3;
ins(3).system = 2;
ins(3).input = 4;
ins(4).system = 2;
ins(4).input = 5;
sys = mimoFeedback(lhand_motion,sys,[],[],ins,outs);
clear ins;

ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 3;
ins(3).system = 2;
ins(3).input = 4;
sys = mimoFeedback(rhand_motion,sys,[],[],ins,outs);
clear ins;

ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 3;
sys = mimoFeedback(pelvis_motion,sys,[],[],ins,outs);
clear ins;

ins(1).system = 2;
ins(1).input = 1;
sys = mimoFeedback(torso_motion,sys,[],[],ins,outs);
clear ins;

qt = QTrajEvalBlock(r,ctrl_data);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);


S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

traj = simulate(sys,[0 ts(end)+1],xtraj.eval(0));
playback(v,traj,struct('slider',true));

end
