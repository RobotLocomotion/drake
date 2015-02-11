function runAtlasBalancingWithContactSensor(use_mex)

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'ZMP'));
import atlasControllers.*;

% put robot in a random x,y,yaw position and balance for 2 seconds
visualize = true;

if (nargin<1); use_mex = true; end
if (nargin<2); use_angular_momentum = false; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

r_pure = r;
l_foot_body = findLinkId(r,'l_foot');
l_foot_frame = RigidBodyFrame(l_foot_body,zeros(3,1),zeros(3,1),'l_foot');
l_foot_force_sensor = ContactForceTorqueSensor(r, l_foot_frame);
r = addSensor(r, l_foot_force_sensor);
r_foot_body = findLinkId(r,'r_foot');
r_foot_frame = RigidBodyFrame(r_foot_body,zeros(3,1),zeros(3,1),'r_foot');
r_foot_force_sensor = ContactForceTorqueSensor(r, r_foot_frame);
r = addSensor(r, r_foot_force_sensor);

r = compile(r);


nq = getNumPositions(r);

% set initial state to fixed point
load('data/atlas_fp.mat');
xstar(1) = 0.1*randn();
xstar(2) = 0.1*randn();
xstar(6) = pi*randn();
initstate = zeros(r.getNumStates, 1);
initstate(1:length(xstar)) = xstar;
r = r.setInitialState(initstate);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

com = getCOM(r,kinsol);

% build TI-ZMP controller
footidx = [findLinkId(r,'r_foot'), findLinkId(r,'l_foot')];
foot_pos = terrainContactPositions(r,kinsol,footidx);
comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
limp = LinearInvertedPendulum(com(3));
[~,V] = lqr(limp,comgoal);

foot_support = RigidBodySupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));


ctrl_data = QPControllerData(false,struct(...
  'acceleration_input_frame',atlasFrames.AtlasCoordinates(r),...
  'D',-com(3)/9.81*eye(2),...
  'Qy',eye(2),...
  'S',V.S,...
  's1',zeros(4,1),...
  's2',0,...
  'x0',[comgoal;0;0],...
  'u0',zeros(2,1),...
  'y0',comgoal,...
  'qtraj',x0(1:nq),...
  'support_times',0,...
  'supports',foot_support,...
  'mu',1.0,...
  'ignore_terrain',false,...
  'constrained_dofs',[]));

% instantiate QP controller
options.slack_limit = 30.0;
options.w_qdd = 0.001*ones(nq,1);
options.w_grf = 0;
options.w_slack = 0.001;
options.debug = false;
options.use_mex = use_mex;

if use_angular_momentum
  options.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  options.W_kdot = 1e-5*eye(3); % angular momentum weight
else
  options.W_kdot = zeros(3);
end

qp = QPController(r,{},ctrl_data,options);
clear options;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 2;
ins(2).system = 1;
ins(2).input = 3;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
outs(3).system = 2;
outs(3).output = 3;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins;

% feedback foot contact detector with QP/atlas
options.use_lcm=false;
options.contact_threshold = 0.002;
fc = FootContactBlock(r_pure,ctrl_data,options);
ins(1).system = 2;
ins(1).input = 1;
sys = mimoFeedback(fc,sys,[],[],ins,outs);
clear ins; 

% feedback PD trajectory controller
options.use_ik = false;
pd = IKPDBlock(r_pure,ctrl_data,options);
ins(1).system = 1;
ins(1).input = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins;

qt = QTrajEvalBlock(r_pure,ctrl_data);
sys = mimoFeedback(qt,sys,[],[],[],outs);

if visualize
  v = r.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  output_select(2).system=1;
  output_select(2).output=2;
  output_select(3).system=1;
  output_select(3).output=3;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0 = initstate;
x0(3) = 1.0; % drop it a bit

traj = simulate(sys,[0 2.0],x0);
if visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end

end
