function runValkyrieWalking(robot_options, walking_options)
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin < 1; robot_options = struct(); end;
if nargin < 2; walking_options = struct(); end;

robot_options = applyDefaults(robot_options, struct('use_bullet', true,...
                                                    'terrain', RigidBodyFlatTerrain,...
                                                    'floating', true,...
                                                    'ignore_self_collisions', true,...
                                                    'ignore_friction', true,...
                                                    'enable_fastqp', false,...
                                                    'dt', 0.001));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Valkyrie(fullfile(getDrakePath,'examples','Valkyrie','urdf','urdf','valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf'),robot_options);
fixed_point_file = fullfile(getDrakePath,'examples','Valkyrie','data','valkyrie_fp_june2015_30joints_one_neck.mat');



r.fixed_point_file = fixed_point_file;

r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

walking_options = applyDefaults(walking_options, struct('initial_pose', [],...
                                                        'navgoal', [1.5;0;0;0;0;0],...
                                                        'max_num_steps', 6,...
                                                        'rms_com_tolerance', 0.0051));
walking_options = applyDefaults(walking_options, r.default_footstep_params);
walking_options = applyDefaults(walking_options, r.default_walking_params);

% set initial state to fixed point
load(fixed_point_file);
if ~isempty(walking_options.initial_pose), xstar(1:6) = walking_options.initial_pose; end
xstar = r.resolveConstraints(xstar);
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.01;

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
r.default_qp_input = valkyrieControllers.QPInputConstantHeight();
qp_link_names.r_foot_name = 'rightFoot+rightLegSixAxis_Frame+rightCOP_Frame';
qp_link_names.l_foot_name = 'leftFoot+leftLegSixAxis_Frame+leftCOP_Frame';
qp_link_names.pelvis_name = 'pelvis+leftPelvisIMU_Frame+rightPelvisIMU_Frame';
qp_link_names.r_knee_name = 'rightKneePitch';
qp_link_names.l_knee_name = 'leftKneePitch';
qp_link_names.l_akx_name = 'leftAnkleRoll';
qp_link_names.r_akx_name = 'rightAnkleRoll';
qp_link_names.r_aky_name = 'rightAnklePitch';
qp_link_names.l_aky_name = 'leftAnklePitch';
r.rpc = valkyrieUtil.propertyCache(r);
walking_plan_data = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan, qp_link_names);

[ytraj, com, rms_com] = valkyrieUtil.simulateWalking(r, walking_plan_data);

v.playback(ytraj, struct('slider', true));

if ~rangecheck(rms_com, 0, walking_options.rms_com_tolerance);
  error('Drake:runAtlasWalkingSplit:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
end


