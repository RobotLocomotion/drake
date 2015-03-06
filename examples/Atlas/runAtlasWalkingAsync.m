function runAtlasWalkingAsync(example_options)
%NOTEST 

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_mex', true,...
                                                        'use_bullet', false,...
                                                        'navgoal', [0.5;0;0;0;0;0],...
                                                        'quiet', true,...
                                                        'num_steps', 4,...
                                                        'terrain', RigidBodyFlatTerrain));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.001;
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

x0 = xstar;

% Find the initial positions of the feet
R=rotz(example_options.navgoal(6));

rfoot_navgoal = example_options.navgoal;
lfoot_navgoal = example_options.navgoal;

rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = r.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', struct('max_num_steps', example_options.num_steps)));

walking_plan_data = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan);
% walking_plan_data = StandingPlan.from_standing_state(x0, r);
planeval = atlasControllers.AtlasPlanEval(r, walking_plan_data);

% Make our Atlas listen for ATLAS_COMMAND over LCM and publish EST_ROBOT_STATE
r = r.setInputFrame(drcFrames.AtlasInput(r));
r = r.setOutputFrame(drcFrames.AtlasState(r));

% Wrap the planeval in a Drake system
planeval_sys = atlasControllers.AtlasPlanEvalAndControlSystem(r, [], planeval);
planeval_sys = planeval_sys.setInputFrame(r.getOutputFrame);

sys = cascade(r, planeval_sys);

output_select(1).system=1;
output_select(1).output=1;
v = v.setInputFrame(sys.getOutputFrame());
sys = mimoCascade(sys,v,[],[],output_select);

T = min(walking_plan_data.duration + 1, 30);

runLCM(sys, x0, struct('tspan', [0, T],...
                        'timekeeper', ''));

v.playback(ytraj, struct('slider', true));