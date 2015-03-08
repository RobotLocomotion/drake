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



% start the controller

control = atlasControllers.InstantaneousQPController(r, [],...
   struct('use_mex', example_options.use_mex));
control.quiet = example_options.quiet;

t0 = 0;
x0 = xstar;
contact_sensor = zeros(length(r.getManipulator().body), 1);

state_coder = drcFrames.AtlasState(r).lcmcoder;
lc = lcm.lcm.LCM.getSingleton();
input_monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input, 'timestamp');
state_monitor = drake.util.MessageMonitor(drake.robot_state_t, 'utime');
lc.subscribe('QP_CONTROLLER_INPUT', input_monitor);
lc.subscribe('EST_ROBOT_STATE', state_monitor);
disp('controller ready');

% block until we get a *new* qp_input
while isempty(input_monitor.getNextMessage(10))
end

while true
  state_msg_data = state_monitor.getNextMessage(10);
  qp_input_msg_data = input_monitor.getMessage();
  if isempty(state_msg_data)
    continue
  end
  [x, t] = state_coder.decode(drc.robot_state_t(state_msg_data));
  qp_input = atlasControllers.QPInput2D.from_lcm(drake.lcmt_qp_controller_input(qp_input_msg_data));
  [~] = instantaneousQPControllermex(control.mex_ptr,...
                                t,...
                                x,...
                                qp_input,...
                                contact_sensor);
end




