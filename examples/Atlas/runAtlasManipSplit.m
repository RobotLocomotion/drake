function runAtlasManipSplit(example_options)
%NOTEST 
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.
% @option use_mex [1] whether to use mex. 0: no, 1: yes, 2: compare mex and non-mex
% @option use_bullet [false] whether to use bullet for collision detect
% @option quiet [true] whether to silence timing printouts

% 
% this function is tested in test/testSplitWalking.m

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_mex', true,...
                                                        'use_bullet', false,...
                                                        'quiet', true,...
                                                        'terrain', RigidBodyFlatTerrain));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.002;
options.terrain = example_options.terrain;
options.use_bullet = example_options.use_bullet;
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
xstar = r.resolveConstraints(xstar);
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.01;

nq = getNumPositions(r);
nv = getNumVelocities(r);

x0 = xstar;
q0 = x0(1:nq);
v0 = x0(nq+(1:nv));
% Plan footsteps to the goal
pelvis = r.findLinkId('pelvis');
cnstr = fix_feet_cnstr(r,x0);
r_hand = r.findLinkId('r_hand');
r_hand_pt = [0;-0.2;0];
kinsol0 = r.doKinematics(q0,v0);
r_hand_pos0 = r.forwardKin(kinsol0,r_hand,r_hand_pt,2);
r_hand_rpy_des = rotmat2rpy(rotx(pi)*quat2rotmat(r_hand_pos0(4:7)));
r_hand_cnstr = {WorldPositionConstraint(r,r_hand,r_hand_pt,r_hand_pos0(1:3)+[0.3;-0.1;0.5],r_hand_pos0(1:3)+[0.5;0.1;0.6],[1,1]),...
  WorldEulerConstraint(r,r_hand,r_hand_rpy_des,r_hand_rpy_des,[1,1])};
qtraj0 = PPTrajectory(zoh([0,1],[q0 q0]));
t_plan = [0,0.5,1];
[xtraj,info] = inverseKinTraj(r,t_plan,qtraj0,qtraj0,cnstr{:},r_hand_cnstr{:});

n_breaks = 20;
t_breaks = linspace(t_plan(1),t_plan(end)*4,n_breaks);
r_hand_breaks = zeros(6,n_breaks);
x_breaks = xtraj.eval(t_breaks);
qtraj_pp = spline(t_breaks,[zeros(nq,1) x_breaks(1:nq,:) zeros(nq,1)]);
for i = 1:n_breaks
  kinsol = r.doKinematics(x_breaks(1:nq,i),x_breaks(nq+(1:nv),i));
  r_hand_breaks(:,i) = r.forwardKin(kinsol,r_hand,zeros(3,1),1);
end
rhand_pp = zoh(t_breaks,r_hand_breaks);
rhand_coefs = reshape(rhand_pp.coefs,6,n_breaks-1,1);
rhand_coefs = [rhand_coefs rhand_coefs(:,end,:)];
rhand_coefs = cat(3,zeros(6,n_breaks,3),rhand_coefs);
link_constraints(2) = struct('link_ndx',r_hand,...
                             'pt',zeros(3,1),...
                             'ts',t_breaks,...
                             'coefs',rhand_coefs);
link_constraints(1) = struct('link_ndx',pelvis,...
                             'pt',zeros(3,1),...
                             'ts',t_breaks,...
                             'coefs',cat(3,zeros(6,n_breaks,3),reshape(x_breaks(1:6,:),6,n_breaks,1)));
manip_plan_data = QPLocomotionPlan.from_configuration_traj(r,qtraj_pp,link_constraints);
% walking_plan_data = StandingPlan.from_standing_state(x0, r);

control = atlasControllers.InstantaneousQPController(r, [],...
   struct('use_mex', example_options.use_mex));
control.quiet = example_options.quiet;
planeval = atlasControllers.AtlasPlanEval(r, manip_plan_data);

plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);
plancontroller.quiet = example_options.quiet;

sys = feedback(r, plancontroller);
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

T = min(manip_plan_data.duration + 1, 30);

% profile on
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
% profile viewer

v.playback(ytraj, struct('slider', true));
end

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
