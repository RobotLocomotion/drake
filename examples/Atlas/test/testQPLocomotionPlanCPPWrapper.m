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
load(r.fixed_point_file);
q = xstar(1 : r.getNumPositions());
v = xstar((r.getNumPositions() + 1) : end);


load('fromBipedFootAndZMPKnots.mat');
settings = QPLocomotionPlanSettings.fromBipedFootAndZMPKnots(foot_motion_data, zmp_knots, r, x0, options);
plan = QPLocomotionPlanCPPWrapper(settings);
plan_matlab = QPLocomotionPlan.from_biped_foot_and_zmp_knots(foot_motion_data, zmp_knots, r, x0, options);

valuecheck(plan.start_time, []);

t_globals = linspace(0, 4, 100);
for t_global = t_globals
%   q = zeros(r.getNumPositions(), 1);
%   q(1) = 2;
%   v = zeros(r.getNumVelocities(), 1);
%   contact_force_detected = rand(r.getNumBodies(), 1) > 0.5;
  contact_force_detected = false(r.getNumBodies(), 1);
  contact_force_detected(r.findLinkId('r_foot')) = true;
  contact_force_detected(r.findLinkId('l_foot')) = true;
  x = [q; v];
  data = plan.getQPControllerInput(t_global, x, contact_force_detected);
  msg = drake.lcmt_qp_controller_input(data);
  valuecheck(plan.start_time, t_globals(1), 1e-10);
  
  rpc = atlasUtil.propertyCache(r);
  input_matlab = plan_matlab.getQPControllerInput(t_global, x, contact_force_detected, rpc);
  msg_matlab = drake.lcmt_qp_controller_input(encodeQPInputLCMMex(input_matlab, false));
  compareLCMMsgs(msg, msg_matlab);
end

next_plan = plan.getSuccessor();
valuecheck(next_plan.duration(), inf);
[~] = plan.getQPControllerInput(t_global, x, contact_force_detected);
valuecheck(next_plan.start_time, plan.start_time, 1e-10);
valuecheck(next_plan.duration, plan.duration, 1e-10);
end
