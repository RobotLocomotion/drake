function runAtlasHandControl(use_mex)

% Tests out the piping to add a robotiq hand (with kinematic loops)
% to Atlas, and hook up a controller to command its joints

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary({fullfile(getDrakePath,'examples','ZMP'),...
                                fullfile(getDrakePath,'examples','Atlas','controllers'),...
                                fullfile(getDrakePath,'examples','Atlas','frames')});

% put robot in a random x,y,yaw position and balance for 2 seconds
visualize = true;

if (nargin<1); use_mex = true; end
if (nargin<2); use_angular_momentum = false; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
options.hands = 'robotiq_weight_only';
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
options.hands = 'robotiq';
r_hands = Atlas('urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r_hands = r_hands.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);
r_hands = compile(r_hands);

nq = getNumPositions(r);

% set initial state to fixed point
load('data/atlas_fp.mat');
xstar(1) = 0; %0.1*randn();
xstar(2) = 0; %0.1*randn();
xstar(6) = 0;% pi*randn();
r = r.setInitialState(xstar);

% and hand state to a feasible sol
% so lcp has an easier time
%load('data/robotiq_feas.mat');
xstar_hands = r_hands.getInitialState();
xstar_hands(1:length(xstar)) = xstar;
%xstar_hands = r_hands.getStateFrame().mergeCoordinates({xstar,xstar_hand});
xstar_hands = r_hands.resolveConstraints(xstar_hands);
r_hands = r_hands.setInitialState(xstar_hands);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

com = getCOM(r,kinsol);

% build TI-ZMP controller
footidx = [findLinkInd(r,'r_foot'), findLinkInd(r,'l_foot')];
foot_pos = terrainContactPositions(r,kinsol,footidx);
comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
limp = LinearInvertedPendulum(com(3));
[~,V] = lqr(limp,comgoal);

foot_support = RigidBodySupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));

% generate manip plan
rhand_ind = findLinkInd(r,'r_hand');
lhand_ind = findLinkInd(r,'l_hand');
rhand_pos = forwardKin(r,kinsol,rhand_ind,[0;0;0],1);
lhand_pos = forwardKin(r,kinsol,lhand_ind,[0;0;0],1);
diff = [0.1+0.1*rand(); 0.05*randn(); 0.1+0.5*rand()];
rhand_goal = rhand_pos(1:3) + diff;
lhand_goal = lhand_pos(1:3) + diff;

rfoot_ind = findLinkInd(r,'r_foot');
lfoot_ind = findLinkInd(r,'l_foot');
rfoot_pos = forwardKin(r,kinsol,rfoot_ind,[0;0;0],1);
lfoot_pos = forwardKin(r,kinsol,lfoot_ind,[0;0;0],1);

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
options.Q = diag(cost(1:r.getNumPositions));
options.quastiStaticFlag = true;

% time spacing of samples for IK
T = 2;
ts = 0:0.1:T;

rhand_traj = PPTrajectory(spline([0 T],[rhand_pos(1:3) rhand_goal]));
lhand_traj = PPTrajectory(spline([0 T],[lhand_pos(1:3) lhand_goal]));
q = zeros(r.getNumPositions,length(ts));
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    rfoot_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,rfoot_ind,[0;0;0],rfoot_pos(1:3),rfoot_pos(1:3)),...
      constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldEulerConstraintType,...
      r.getMexModelPtr,rfoot_ind,rfoot_pos(4:6),rfoot_pos(4:6))};
    lfoot_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,lfoot_ind,[0;0;0],lfoot_pos(1:3),lfoot_pos(1:3)),...
      constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldEulerConstraintType,...
      r.getMexModelPtr,lfoot_ind,lfoot_pos(4:6),lfoot_pos(4:6))};
    rhand_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,rhand_ind,[0;0;0],rhand_traj.eval(t),rhand_traj.eval(t))};
    lhand_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,lhand_ind,[0;0;0],lhand_traj.eval(t),lhand_traj.eval(t))};
    ikoptions = IKoptions(r);
    ikoptions = ikoptions.setQ(options.Q);
    q(:,i) = inverseKin(r,q(:,i-1),q(:,i-1), ...
      rfoot_cnst{:},lfoot_cnst{:},rhand_cnst{:},lhand_cnst{:},ikoptions);
  else
    q = q0;
  end
end

qtraj = PPTrajectory(spline(ts,q));

ctrl_data = QPControllerData(false,struct(...
  'acceleration_input_frame',AtlasCoordinates(r),...
  'D',-com(3)/9.81*eye(2),...
  'Qy',eye(2),...
  'S',V.S,...
  's1',zeros(4,1),...
  's2',0,...
  'x0',[comgoal;0;0],...
  'u0',zeros(2,1),...
  'y0',comgoal,...
  'qtraj',qtraj,...
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
ins(3).system = 2;
ins(3).input = 2;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
sys = mimoFeedback(qp,r_hands,[],[],ins,outs);
clear ins;
clear outs;

rh = RobotiqControlBlock(r_hands, 2, '');
ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 1;
ins(2).input = 2;
outs(1).system = 1;
outs(1).output = 1;
outs(2).system = 1;
outs(2).output = 2;
sys = mimoFeedback(sys, rh,[],[],ins,outs);
clear ins;
clear outs;

% feedback foot contact detector with QP/atlas
options.use_lcm=false;
options.contact_threshold = 0.002;
fc = FootContactBlock(r,ctrl_data,options);
ins(1).system = 2;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
sys = mimoFeedback(fc,sys,[],[],ins,outs);
clear ins;

% feedback PD trajectory controller
options.use_ik = false;
pd = IKPDBlock(r,ctrl_data,options);
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins;

qt = QTrajEvalBlock(r,ctrl_data);
sys = mimoFeedback(qt,sys,[],[],[],outs);

if visualize
  v = r_hands.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  output_select(2).system=1;
  output_select(2).output=2;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0 = xstar_hands;
x0(3) = 1.0; % drop it a bit

traj = simulate(sys,[0 2],x0);
if visualize
  % This doesn't see hand movements. Why?
  playback(v,traj,struct('slider',true));
end

%xf = traj.eval(traj.tspan(2));

end
