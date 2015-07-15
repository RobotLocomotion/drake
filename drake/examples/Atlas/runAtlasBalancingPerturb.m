function runAtlasBalancingPerturb(example_options) 
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. Perturbs a supplied amount at
% a specified time.
% @option use_mex [1] whether to use mex. 0: no, 1: yes, 2: compare mex and non-mex
% @option use_bullet [false] whether to use bullet for collision detect
% @option perturb_body ['pelvis']
% @option perturb_amount [1000; 0; 0] applied to pelvis
% @option perturb_timing [0.1 0.2] (i.e. tstart tend)

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_bullet', false,...
                                                        'perturb_body', 'mtorso',...
                                                        'perturb_amount', [250;0;0],...
                                                        'perturb_timing', [1.0 1.1],...
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
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_convex_hull.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

options_complete = options;
options_complete.external_force = example_options.perturb_body;
r_complete = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_convex_hull.urdf'),options_complete);
r_complete = r_complete.removeCollisionGroupsExcept({'heel','toe'});
r_complete = compile(r_complete);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
xstar(3) = xstar(3)+.05;
xstar = r.resolveConstraints(xstar);
xstar_complete = zeros(r_complete.getNumStates(), 1);
xstar_complete(1:size(xstar,1)) = xstar;
xstar_complete = r_complete.resolveConstraints(xstar_complete);
r_complete = r_complete.setInitialState(xstar_complete);
r = r.setInitialState(xstar);
v = r_complete.constructVisualizer;
v.display_dt = 0.01;
nq = getNumPositions(r);
x0 = xstar;


% Construct plan
settings = QPLocomotionPlanSettings.fromStandingState(x0, r);
% settings.planned_support_command = QPControllerPlan.support_logic_maps.kinematic_or_sensed; % Only use supports when in contact
standing_plan = QPLocomotionPlanCPPWrapper(settings);

control = atlasControllers.InstantaneousQPController(r, [], struct());
planeval = atlasControllers.AtlasPlanEval(r, standing_plan);

plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);

T = 6;
ts = example_options.perturb_timing;
ts = [0 ts(1) ts(2) T];
vals = example_options.perturb_amount;
vals = [zeros(3, 1) vals zeros(3, 1) zeros(3, 1)]; 
force_application = zoh(ts, vals);
% Pass through outputs from robot
outs(1).system = 1;
outs(1).output = 1;
outs(2).system = 1;
outs(2).output = 2;
sys = mimoFeedback(r_complete, plancontroller, [], [], [], outs);
sys = mimoCascade(setOutputFrame(PPTrajectory(force_application), r_complete.getInputFrame.frame{2}), sys);
output_select(1).system=1;
output_select(1).output=1;
output_select(2).system=1;
output_select(2).output=2;
sys = mimoCascade(sys,v,[],[],output_select);



% profile on
ytraj = simulate(sys, [0, T], xstar_complete, struct('gui_control_interface', true));
% profile viewer

v.playback(ytraj, struct('slider', true));

xf = ytraj.eval(ytraj.tspan(end));
kinsol = doKinematics(r, xf(1:nq));
comf = getCOM(r, kinsol);

% Make sure we're still standing
rangecheck(comf, [-0.02; -0.02; 0.9], [0.02; 0.02; inf]);
