function runAtlasManip(example_options)
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.
% This function is intended to test the singularity that would encounter if
% we used Euler angle. We should observe vey smooth output motion as we are
% tracking the spatial velocity.
% @option use_bullet [false] whether to use bullet for collision detect

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_bullet', false,...
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

plan_settings = QPLocomotionPlanSettings.fromQuasistaticQTraj(r,PPTrajectory(qtraj_pp),...
  struct('bodies_to_track',[pelvis,r_hand],...
         'quat_task_to_world',repmat(uniformlyRandomQuat(),1,2),...
         'translation_task_to_world',randn(3,2),...
         'track_com_traj',true));
plan_settings.gain_set = 'manip';
r_arm_idx = r.findPositionIndices('r_arm');
plan_settings.untracked_joint_inds = r_arm_idx;
manip_plan_data = QPLocomotionPlanCPPWrapper(plan_settings);

%manip_plan_data = QPLocomotionPlan.from_quasistatic_qtraj(r,PPTrajectory(qtraj_pp),...
%  struct('bodies_to_track',[pelvis,r_hand],...
%         'quat_task_to_world',repmat(uniformlyRandomQuat(),1,2),...
%         'translation_task_to_world',randn(3,2),...
%         'track_com_traj',true));

control = atlasControllers.InstantaneousQPController(r, []);
planeval = atlasControllers.AtlasPlanEval(r, manip_plan_data);
plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);
sys = feedback(r, plancontroller);
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

T = min(manip_plan_data.duration + 1, 30);
% profile on
tic
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
toc
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
