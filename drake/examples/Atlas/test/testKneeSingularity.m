function testKneeSingularity()
% Make sure we can approach and leave the knee singularity gracefully by commanding a set of poses which are unreachable for the leg. 

checkDependency('iris');
checkDependency('lcmgl');
path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'Atlas'));

robot_options = struct();
robot_options = applyDefaults(robot_options, struct('use_bullet', true,...
                                                    'terrain', RigidBodyFlatTerrain,...
                                                    'floating', true,...
                                                    'ignore_self_collisions', true,...
                                                    'ignore_friction', true,...
                                                    'hand_right', 'robotiq_weight_only',...
                                                    'hand_left', 'robotiq_weight_only',...
                                                    'dt', 0.001));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),robot_options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
xstar = r.resolveConstraints(xstar);

r = r.setInitialState(xstar);
x0 = xstar;
nq = r.getNumPositions();

v = r.constructVisualizer();
v.display_dt = 0.05;
v.draw(0, x0);

kinsol = r.doKinematics(x0(1:nq));
com0 = r.getCOM(kinsol);
lfoot_pose = r.forwardKin(kinsol, r.foot_frame_id.left, [0;0;0], 2);
lfoot_poses = [lfoot_pose,...
               lfoot_pose,...
               lfoot_pose + [0;0;0.05;0;0;0;0],...
               [1;0.1;0.2;expmap2quat([0;-pi/2;0])],...
               [0;0.1;0.2;1;0;0;0],...
               [-1;0.1;0.2;expmap2quat([0;pi/2;0])],...
               [0;0.1;0.2;1;0;0;0],...
               [0;1;0.2;rpy2quat([0;-pi/2;pi/2])],...
               [0;0.1;0.2;1;0;0;0],...
               lfoot_pose + [0;0;0.05;0;0;0;0],...
               lfoot_pose,...
               lfoot_pose,...
               lfoot_pose,...
               lfoot_pose];

dt = 2;
foot_ts = [0, 2, 4, 4+dt*(1:(size(lfoot_poses,2)-3))];
nposes = length(foot_ts);
lfoot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.left, foot_ts, lfoot_poses);
lfoot_motion.in_floating_base_nullspace = true(1,nposes);
lfoot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];

rfoot_pose = r.forwardKin(kinsol, r.foot_frame_id.right, [0;0;0], 2);
rfoot_poses = repmat(rfoot_pose, 1, nposes);
rfoot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.right, foot_ts, rfoot_poses);
rfoot_motion.in_floating_base_nullspace = true(1,nposes);
rfoot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];


double_support = RigidBodySupportState(r, [r.foot_body_id.left, r.foot_body_id.right]);
zmp_knots = struct('t', num2cell(foot_ts([1,2,end-3:end])),...
                   'zmp', {com0(1:2), rfoot_pose(1:2), rfoot_pose(1:2), rfoot_pose(1:2), com0(1:2), com0(1:2)},...
                   'supp', {double_support, r.right_full_support, r.right_full_support, double_support, double_support, double_support});

plan_settings = QPLocomotionPlanSettings.fromBipedFootAndZMPKnots([rfoot_motion, lfoot_motion], zmp_knots, r, x0);
plan_settings.gain_set = 'walking';
plan = QPLocomotionPlanCPPWrapper(plan_settings);

% Build our controller and plan eval objects
control = atlasControllers.InstantaneousQPController(r, []);
planeval = atlasControllers.AtlasPlanEval(r, plan);
plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);
sys = feedback(r, plancontroller);

% Add a visualizer
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

% Simulate and draw the result
T = plan.duration + 1;
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
[com, rms_com] = atlasUtil.plotWalkingTraj(r, ytraj, plan);

v.playback(ytraj, struct('slider', true));

if ~rangecheck(rms_com, 0, 0.005);
  error('Drake:testKneeSingularity:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
end

