function testQPLocomotionPlanSettings()
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
qp_locomotion_plan_ptr = constructQPLocomotionPlanmex(r.getManipulator().mex_model_ptr, settings, 'qp_controller_input');

t_global = 3;
q = randn(r.getNumPositions(), 1);
v = randn(r.getNumVelocities(), 1);
contact_force_detected = rand(r.getNumBodies(), 1) > 0.5;
for i = 1 : 100
  qpLocomotionPlanPublishQPControllerInputmex(qp_locomotion_plan_ptr, t_global, q, v, contact_force_detected);
  pause(0.01)
end
end
