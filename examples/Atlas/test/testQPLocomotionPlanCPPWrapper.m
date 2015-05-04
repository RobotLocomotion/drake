function testQPLocomotionPlanCPPWrapper()
path = addpathTemporary(fullfile(getDrakePath,'examples','Atlas'));

robot_options = struct(...
  'use_bullet', true,...
  'terrain', RigidBodyFlatTerrain,...
  'floating', true,...
  'ignore_self_collisions', true,...
  'ignore_friction', true,...
  'enable_fastqp', false,...
  'use_new_kinsol', true,...
  'dt', 0.001);
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'), robot_options);

load('fromBipedFootAndZMPKnots.mat');
settings = QPLocomotionPlanSettings.fromBipedFootAndZMPKnots(foot_motion_data, zmp_knots, r, x0, options);
settings.untracked_joint_inds = 20;

plan = QPLocomotionPlanCPPWrapper(settings);
valuecheck(plan.start_time, []);

t_globals = linspace(3, 4, 100);
for t_global = t_globals
  q = randn(r.getNumPositions(), 1);
  v = randn(r.getNumVelocities(), 1);
  contact_force_detected = rand(r.getNumBodies(), 1) > 0.5;
  x = [q; v];
  data = plan.getQPControllerInput(t_global, x, contact_force_detected);
  valuecheck(plan.start_time, t_globals(1), 1e-10);
  typecheck(data, 'char');
end

next_plan = plan.getSuccessor();
valuecheck(next_plan.duration(), inf);
[~] = plan.getQPControllerInput(t_global, x, contact_force_detected);
valuecheck(next_plan.start_time, plan.start_time, 1e-10);
valuecheck(next_plan.duration, plan.duration, 1e-10);
end
