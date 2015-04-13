function runAtlasPelvisDrop(robot_options, walking_options)
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.

checkDependency('gurobi');
checkDependency('lcmgl');

path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'Atlas'));

if nargin < 1; robot_options = struct(); end;
if nargin < 2; walking_options = struct(); end;

robot_options = applyDefaults(robot_options, struct('use_bullet', false,...
                                                    'terrain', RigidBodyFlatTerrain,...
                                                    'floating', true,...
                                                    'ignore_self_collisions', true,...
                                                    'ignore_friction', true,...
                                                    'use_new_kinsol', true,...
                                                    'dt', 0.001));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),robot_options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

walking_options = applyDefaults(walking_options, struct('initial_pose', [],...
                                                        'navgoal', [1.5;0;0;0;0;0],...
                                                        'max_num_steps', 6));
walking_options = applyDefaults(walking_options, r.default_footstep_params);
walking_options = applyDefaults(walking_options, r.default_walking_params);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
if ~isempty(walking_options.initial_pose), xstar(1:6) = walking_options.initial_pose; end
xstar = r.resolveConstraints(xstar);
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.05;

nq = getNumPositions(r);

x0 = xstar;

% Find the initial positions of the feet
R=rotz(walking_options.navgoal(6));

rfoot_navgoal = walking_options.navgoal;
lfoot_navgoal = walking_options.navgoal;

rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = r.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', walking_options));
for j = 1:length(footstep_plan.footsteps)
  footstep_plan.footsteps(j).walking_params = walking_options;
end

% Generate a dynamic walking plan
walking_plan_data = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan);

% Manually tweak the pelvis trajectory
j = find([walking_plan_data.body_motions.body_id] == r.findLinkId('pelvis'));
ts = walking_plan_data.body_motions(j).ts;
coefs = walking_plan_data.body_motions(j).coefs;
r0 = coefs(:,1,end);
pp = foh([ts(1), 7.5, 9.5, ts(end)], [r0, r0, r0-[0;0;0.1;0;0;0], r0]);
walking_plan_data.body_motions(j) = BodyMotionData.from_body_xyzrpy_pp(walking_plan_data.body_motions(j).body_id, pp);

% Build our controller and plan eval objects
control = atlasControllers.InstantaneousQPController(r, []);
planeval = atlasControllers.AtlasPlanEval(r, walking_plan_data);
plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);
sys = feedback(r, plancontroller);

% Add a visualizer
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

% Simulate and draw the result
T = min(walking_plan_data.duration + 1, 30);
tic
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
toc
[com, rms_com] = atlasUtil.plotWalkingTraj(r, ytraj, walking_plan_data);

v.playback(ytraj, struct('slider', true));

if ~rangecheck(rms_com, 0, 0.01);
  error('Drake:runAtlasWalkingSplit:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
end


