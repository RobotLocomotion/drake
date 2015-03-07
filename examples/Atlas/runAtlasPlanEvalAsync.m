function runAtlasPlanEvalAsync(example_options)
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

% % set initial state to fixed point
% load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
% if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
% xstar = r.resolveConstraints(xstar);
% r = r.setInitialState(xstar);

state_frame = drcFrames.AtlasState(r);
state_frame.subscribe('EST_ROBOT_STATE');
x0 = [];
while isempty(x0)
  [x0, ~] = state_frame.getNextMessage(50);
end

nq = getNumPositions(r);

navgoal = example_options.navgoal + [x0(1:2); 0;0;0;0];
R=rotz(navgoal(6));

rfoot_navgoal = navgoal;
lfoot_navgoal = navgoal;

rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = r.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', struct('max_num_steps', example_options.num_steps)));

walking_plan = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan);
% walking_plan = StandingPlan.from_standing_state(x0, r);
planeval = atlasControllers.AtlasPlanEval(r, {WaitForRobotStatePlan(), walking_plan});

disp('plan eval ready');
while true
  [x, t] = state_frame.getNextMessage(10);
  qp_input = planeval.getQPControllerInput(t, x);
  if ~isempty(qp_input)
    encodeQPInputLCMMex(qp_input);
  end
end
