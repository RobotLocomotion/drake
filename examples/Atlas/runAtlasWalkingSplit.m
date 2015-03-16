function runAtlasWalkingSplit(example_options)
%NOTEST 
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.
% @option use_mex [1] whether to use mex. 0: no, 1: yes, 2: compare mex and non-mex
% @option use_bullet [false] whether to use bullet for collision detect
% @option navgoal the goal for footstep planning
% @option quiet [true] whether to silence timing printouts
% @option num_steps [4] max number of steps to take
% 
% this function is tested in test/testSplitWalking.m

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
r.warning_manager.warnOnce('Drake:MuAssumedToBe1', 'Setting mu=1 because contactConstraintsBV.m assumes that to be the case');
walking_plan_data.mu = 1;

% walking_plan_data = StandingPlan.from_standing_state(x0, r);

control = atlasControllers.InstantaneousQPController(r, [],...
   struct('use_mex', example_options.use_mex));
control.quiet = example_options.quiet;
planeval = atlasControllers.AtlasPlanEval(r, walking_plan_data);

plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);
plancontroller.quiet = example_options.quiet;

sys = feedback(r, plancontroller);
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

T = min(walking_plan_data.duration + 1, 30);

% profile on
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
% profile viewer

v.playback(ytraj, struct('slider', true));

