function runAtlasHandControl(example_options)

% Tests out the piping to add a robotiq hand (with kinematic loops)
% to Atlas, and hook up a controller to command its joints

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'ZMP'));
import atlasControllers.*;

% put robot in a random x,y,yaw position and balance for 2 seconds
if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_mex', true, ...
                                                        'use_bullet', false,...
                                                        'visualize', true,...
                                                        'terrain', RigidBodyFlatTerrain));

if (nargin<1); use_mex = true; end
if (nargin<2); use_angular_momentum = false; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.001;
options.terrain = example_options.terrain;
options.use_bullet = example_options.use_bullet;
options.use_mex = example_options.use_mex;
options.hand_left = 'robotiq_weight_only';
options.hand_right = 'robotiq_weight_only';
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
options.hand_left = 'robotiq_tendons';
options.hand_right = 'robotiq_tendons';
r_hands = Atlas('urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r_hands = r_hands.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);
r_hands = compile(r_hands);

nq = getNumPositions(r);
nv = getNumVelocities(r);

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
v0 = x0(nq+(1:nv));
pelvis = r.findLinkId('pelvis');
cnstr = fix_feet_cnstr(r,x0);
r_hand = r.findLinkId('r_hand');
r_hand_pt = [0;-0.2;0];
kinsol0 = r.doKinematics(q0,v0);
r_hand_pos0 = r.forwardKin(kinsol0,r_hand,r_hand_pt,2);
r_hand_cnstr = {WorldEulerConstraint(r,r_hand,[nan;pi/2;nan],[nan;pi/2;nan],[0.3 0.7]),...
  WorldEulerConstraint(r,r_hand,[nan;pi/4;nan],[nan;0.6*pi;nan],[1,1])};

qtraj0 = PPTrajectory(zoh([0,1],[q0 q0]));
t_plan = [0,0.5,1];
[xtraj,info] = inverseKinTraj(r,t_plan,qtraj0,qtraj0,cnstr{:},r_hand_cnstr{:});

n_breaks = 20;
t_breaks = linspace(t_plan(1),t_plan(end),n_breaks);

x_breaks = xtraj.eval(t_breaks);
t_breaks = t_breaks*4;
x_breaks(nq+(1:nv),:) = 1/4*x_breaks(nq+(1:nv),:);
qtraj_pp = spline(t_breaks,[zeros(nq,1) x_breaks(1:nq,:) zeros(nq,1)]);

manip_plan_data = QPLocomotionPlanSettings.fromQuasistaticQTraj(r,PPTrajectory(qtraj_pp),...
  struct('bodies_to_track',[pelvis,r_hand],...
         'quat_task_to_world',repmat(uniformlyRandomQuat(),1,2),...
         'translation_task_to_world',randn(3,2),...
         'is_quasistatic',true,...
         'gain_set', 'manip'));
r_arm_idx = r.findPositionIndices('r_arm');

manip_plan_data.untracked_joint_inds = r_arm_idx;
control = atlasControllers.InstantaneousQPController(r, []);
planeval = atlasControllers.AtlasPlanEval(r, QPLocomotionPlanCPPWrapper(manip_plan_data));
plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);

ins(1).system = 2;
ins(1).input = 2;
ins(2).system = 2;
ins(2).input = 3;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
outs(3).system = 2;
outs(3).output = 3;
sys = mimoFeedback(plancontroller,r_hands,[],[],ins,outs);
clear ins;
clear outs;

ins(1).system = 1;
ins(1).input = 2;
outs(1).system = 1;
outs(1).output = 1;
outs(2).system = 1;
outs(2).output = 2;
outs(3).system = 1;
outs(3).output = 3;

rh = RobotiqControlBlock(r_hands, 2, 'right_');
sys = mimoFeedback(sys, rh, [], [], ins, outs);

lh = RobotiqControlBlock(r_hands, 3, 'left_');
sys = mimoFeedback(sys, lh, [], [], [], outs);

if example_options.visualize
  v = r_hands.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  output_select(2).system=1;
  output_select(2).output=2;
  output_select(3).system = 1;
  output_select(3).output = 3;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0 = xstar_hands;
T = min(manip_plan_data.duration + 1, 30);
traj = simulate(sys,[0 T],x0);
if example_options.visualize
  % This doesn't see hand movements. Why?
  playback(v,traj,struct('slider',true));
end

%xf = traj.eval(traj.tspan(2));

function cnstr = fix_feet_cnstr(r,x0)
q0 = x0(1:r.getNumPositions);
v0 = x0(r.getNumPositions+(1:r.getNumVelocities));
kinsol0 = r.doKinematics(q0,v0,false);
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_foot_pts = r.getBody(l_foot).getTerrainContactPoints;
r_foot_pts = r.getBody(r_foot).getTerrainContactPoints;
l_foot_pos0 = r.forwardKin(kinsol0,l_foot,zeros(3,1),2);
r_foot_pos0 = r.forwardKin(kinsol0,r_foot,zeros(3,1),2);
lfoot_cnstr = {WorldPositionConstraint(r,l_foot,zeros(3,1),l_foot_pos0(1:3),l_foot_pos0(1:3)),...
  WorldQuatConstraint(r,l_foot,l_foot_pos0(4:7),0)};
rfoot_cnstr = {WorldPositionConstraint(r,r_foot,zeros(3,1),r_foot_pos0(1:3),r_foot_pos0(1:3)),...
  WorldQuatConstraint(r,r_foot,r_foot_pos0(4:7),0)};
qsc = QuasiStaticConstraint(r,[-inf,inf],1);
qsc = qsc.addContact(l_foot,l_foot_pts,r_foot,r_foot_pts);
qsc = qsc.setShrinkFactor(0.6);
qsc = qsc.setActive(true);
utorso = r.findLinkId('utorso');
utorso_upright = WorldGazeDirConstraint(r,utorso,[0;0;1],[0;0;1],0.05*pi);
cnstr = [lfoot_cnstr,rfoot_cnstr,{qsc,utorso_upright}];
end

end
