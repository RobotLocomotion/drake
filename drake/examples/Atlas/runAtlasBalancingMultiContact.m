function runAtlasBalancingMultiContact(example_options)
%NOTEST
% Run the balancing controller on a convex hull model using the new
% multiple_contacts logic to allow us to balance even with part of the foot
% overhanging.
 
checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_mex', true,...
                                                        'use_bullet', false,...
                                                        'navgoal', [1.0;0;0;0;0;0],...
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

box_xpos = 0.19;
boxurdf = fullfile(getDrakePath,'matlab','systems','plants','test','FallingBrick.urdf');
box_r = RigidBodyManipulator();
box_r = box_r.setTerrain(RigidBodyFlatTerrain());
box_r = box_r.addRobotFromURDF(boxurdf, [box_xpos;0;0], [0;0;pi/2]);
box_r = box_r.compile();
height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(box_r,[],-3:.015:3, -3:.015:3, 10);
options.terrain = height_map;

options.use_bullet = true;
options.multiple_contacts = true;
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_convex_hull.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = r.addRobotFromURDF(fullfile(getDrakePath,'matlab','systems','plants','test','FallingBrick.urdf'), [box_xpos;0;0], [0;0;pi/2]);
options.floating = false;

r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
xstar(3) = xstar(3)+.5025;
xstar = r.resolveConstraints(xstar);
r = r.setInitialState(xstar);


v = r.constructVisualizer;
v.display_dt = 0.01;

x0 = xstar;

% Construct plan
settings = QPLocomotionPlanSettings.fromStandingState(x0, r);
settings.planned_support_command = QPControllerPlan.support_logic_maps.kinematic_or_sensed; % Only use supports when in contact
standing_plan = QPLocomotionPlanCPPWrapper(settings);

control = bipedControllers.InstantaneousQPController(r.getManipulator().urdf{1}, r.control_config_file, fullfile(getDrakePath(), 'examples', 'Atlas', 'config', 'urdf_modifications_no_hands.yaml'));
control.quiet = example_options.quiet;
planeval = bipedControllers.BipedPlanEval(r, standing_plan);

plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);
plancontroller.quiet = example_options.quiet;

sys = feedback(r, plancontroller);
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

T = 5;

% profile on
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
% profile viewer

v.playback(ytraj, struct('slider', true));

