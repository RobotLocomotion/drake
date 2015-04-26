function [xtraj,info,v] = testTaskSpaceRRT(options, rng_seed)
% TIMEOUT 600
if nargin < 1 || isempty(options), options = struct(); end
if nargin < 2, rng; else rng(rng_seed); end
w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'planning_mode'), options.planning_mode = 'rrt_connect'; end;
if ~isfield(options,'visualize'), options.visualize = false; end;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;
urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
q_nom = S.xstar(1:nq);
q_zero = zeros(nq, 1);

world = r.findLinkId('world');
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_hand = r.findLinkId('l_hand');
l_foot_pts = r.getBody(l_foot).getTerrainContactPoints();
r_foot_pts = r.getBody(r_foot).getTerrainContactPoints();
joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');

% Add obstacles
collision_object = RigidBodyCapsule(0.05,1,[0.95,0.22,0.35],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[0.95,-0.22,0.35],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[0.8,-0.05,0.35],[-pi/4,0,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[0.45,-0.05,0.35],[-pi/4,0,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[-0.35,-0.27,0],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, l_hand, collision_object);
collision_object = RigidBodyCapsule(0.05,1,[0;0;0],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];

r = r.compile();
warning(w);

xyz_quat_start = [0.5969; -0.1587; 0.85; -0.584; -0.457; 0.662; 0.112];

% IK constraints
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(r_foot, r_foot_pts);

point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [0.99999962214379723, 3.8873668451910772e-05, 0.00086844752325226373, -0.024113362129690341; -4.319650228383918e-05, 0.99998760778828055, 0.0049781928381826216, 0.13142880655433892; -0.00086824324064880729, -0.0049782284710370005, 0.99998723161596681, 0.081845132612297311; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame = inv(ref_frame);
position_constraint_1 = WorldPositionConstraint(r, l_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


quat_constraint_2 = WorldQuatConstraint(r, l_foot, [0.99999680768841015; -0.0024891132733300568; 0.00043417407699420605; -2.0517608182535892e-05], 0.0, [0.0, 1.0]);


point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [0.99999972333813658, -3.8603987442147522e-05, 0.00074285488657430923, -0.024113358389590833; 4.2294235092508014e-05, 0.99998765711726534, -0.0049682818277853539, -0.13142881299268941; -0.00074265392211426647, 0.0049683118717304582, 0.99998738209154281, 0.081845129013906948; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame = inv(ref_frame);
position_constraint_3 = WorldPositionConstraint(r, r_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


quat_constraint_4 = WorldQuatConstraint(r, r_foot, [0.99999684531339206; 0.0024841562616134435; 0.00037137837375452614; 2.0224619435999976e-05], 0.0, [0.0, 1.0]);

posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = q_zero(joint_inds) + [-0.08726646259971647; -0.08726646259971647; -inf];
joints_upper_limit = q_zero(joint_inds) + [0.08726646259971647; 0.08726646259971647; inf];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

% fixed right arm
posture_constraint_7 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_shz; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.neck_ay];
joints_lower_limit = q_nom(joint_inds);
joints_upper_limit = q_nom(joint_inds);
posture_constraint_7 = posture_constraint_7.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.l_leg_kny;joints.r_leg_kny];
joints_lower_limit = 30*pi/180*[1;1];
joints_upper_limit = 120*pi/180*[1;1];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

point_in_link_frame = [0; -0.24449999999999988; 0.011200000000000071];
ref_frame = [0.10040853387866658, 0.30507204666777654, 0.94702121025152886, 0.19671872655867628; -0.070421541493923046, 0.95162340926777023, -0.29908810311880585, 1.0145817508809061; -0.9924509725008871, -0.036659695518642732, 0.11703475512224609, 0.9; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0; -0; -0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_7 = WorldPositionInFrameConstraint(r, l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);
%ref_frame = inv(ref_frame);
%position_constraint_7 = WorldPositionConstraint(r, l_hand, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [1.0, 1.0]);

l_hand_initial_position_constraint = WorldPositionConstraint(r, l_hand, [0;0;0], xyz_quat_start(1:3), xyz_quat_start(1:3));
point_in_link_frame = [-0.35; -0.27; 0.011200000000000071];



quat_constraint_8 = WorldQuatConstraint(r, l_hand, [0.1439; 0.6104; -0.0161; 0.7787], 10*pi/180, [1.0, 1.0]);

l_hand_initial_quat_constraint = WorldQuatConstraint(r, l_hand, xyz_quat_start(4:7), 0*pi/180);

min_distance = 0.01;
active_collision_options.body_idx = setdiff(1:r.getNumBodies(),[l_foot,r_foot]);

options.display_after_every = 100;

TA = TaskSpaceMotionPlanningTree(r, 'l_hand', point_in_link_frame);
TA  = TA.setMinDistance(min_distance);
aabb_pts = r.getBody(TA.end_effector_id).getAxisAlignedBoundingBoxPoints();
aabb_pts = bsxfun(@minus, aabb_pts, TA.end_effector_pt);
max_radius = max(sqrt(sum(aabb_pts.^2,1)));
TA  = TA.setOrientationWeight(2*pi*max_radius);
TA.max_edge_length = 0.2;
TA.max_length_between_constraint_checks = 0.05;
TA.angle_tol = 1*pi/180;
TA.position_tol = 1e-3;
TA.trees{TA.cspace_idx}.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), [l_foot, r_foot]);

ik_seed_pose = q_nom;
ik_nominal_pose = q_nom;
cost = Point(r.getPositionFrame(),10);
for i = r.getNumBodies():-1:1
  if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
    cost(r.getBody(r.getBody(i).parent).position_num) = ...
      cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
  end
end
cost = cost/min(cost);
Q = diag(cost);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setMajorIterationsLimit(100);
ikoptions = ikoptions.setQ(Q);
ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
TA.trees{TA.cspace_idx}.ikoptions = ikoptions;


active_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5,posture_constraint_6, posture_constraint_7,posture_constraint_8, l_hand_initial_position_constraint , l_hand_initial_quat_constraint};
[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

if options.visualize
  v = r.constructVisualizer();
else
  v = [];
end

posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

base_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5,posture_constraint_6,posture_constraint_7,posture_constraint_8};


active_constraints = [base_constraints, {position_constraint_7,quat_constraint_8}];
[q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

if options.visualize
  v.draw(0,q_end);
end
kinsol = r.doKinematics(q_start);
xyz_quat_start = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
kinsol = r.doKinematics(q_end);
xyz_quat_goal = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
if options.visualize
  v.draw(0,q_start);
end

position_constraint_7 = WorldPositionConstraint(r, l_hand, point_in_link_frame, xyz_quat_start(1:3), xyz_quat_start(1:3), [1.0, 1.0]);

quat_constraint_8 = WorldQuatConstraint(r, l_hand, xyz_quat_start(4:7), 0, [1.0, 1.0]);

active_constraints = [base_constraints,{position_constraint_7,quat_constraint_8}];

[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

x_start = [xyz_quat_start;q_start];
x_goal = [xyz_quat_goal;q_end];
xyz_box_edge_length = 2;
xyz_min = min(xyz_quat_start(1:3),xyz_quat_goal(1:3)) - xyz_box_edge_length/2;
xyz_max = max(xyz_quat_start(1:3),xyz_quat_goal(1:3)) + xyz_box_edge_length/2;

TA = TA.setTranslationSamplingBounds(xyz_min, xyz_max);
TA = TA.addKinematicConstraint(base_constraints{:});
TA = TA.setNominalConfiguration(q_nom);

TA = TA.compile();
TB = TA;
assert(TA.checkConstraints(x_start))
assert(TB.checkConstraints(x_goal))

% n_ee_poses_tried = 1;
%sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:},collision_constraint);
% sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:});
% sample_prog = sample_prog.setQ(0.1*ikoptions.Q);
% sample_prog = sample_prog.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_IterationsLimit);
% sample_prog.setSolverOptions('snopt','MajorFeasibilityTolerance',ikoptions.SNOPT_MajorFeasibilityTolerance);
% sample_prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
TA = TA.setLCMGL('TA',[1,0,0]);
%TA.TB = TA.TB.setLCMGL('TA.TB',[1,0,0]);
TB = TB.setLCMGL('TB',[0,0,1]);
rrt_timer = tic;
display('About to plan ...')
switch options.planning_mode
  case 'rrt'
    [TA, path_ids_A, info] = TA.rrt(x_start, x_goal, TA, options);
  case 'rrt_connect'
    display('Running RRTConnect')
    [TA, path_ids_A, info, TB] = TA.rrtConnect(x_start, x_goal, TB, options);
end
rrt_time = toc(rrt_timer);
fprintf('Timing:\n');
fprintf('  RRT:       %5.2f s\n', rrt_time);

T_smooth = TA;
% interp_weight determines how much consideration is given to joint
% space distance during smoothing:
%  * 0 - end-effector distance only
%  * 1 - joint-space distance only
T_smooth.interp_weight = 0.5;
q_idx = TA.idx{TA.cspace_idx};

if (info == 1) && (options.n_smoothing_passes > 0)
  % Factor in joint-space distance in smoothing because its easy for Atlas v5 to
  % have configurations in which the hand poses are almost identical but the
  % joint poses are very different.
  T_smooth.weights(T_smooth.cspace_idx) = 0.5;
  smoothing_timer = tic;
  T_smooth = T_smooth.setLCMGL('T_smooth', TA.line_color);
  [T_smooth, id_last] = T_smooth.recursiveConnectSmoothing(path_ids_A, options.n_smoothing_passes, options.visualize);
  path_ids_A = T_smooth.getPathToVertex(id_last);
  smoothing_time = toc(smoothing_timer);
  fprintf('  Smoothing: %5.2f s\n', smoothing_time);
  if options.visualize
    drawTree(TA);
    drawTree(TB);
    drawPath(T_smooth, path_ids_A);
  end
end

q_path = extractPath(T_smooth, path_ids_A);
path_length = size(q_path,2);

% Scale timing to obey joint velocity limits
% Create initial spline
q_traj = PPTrajectory(pchip(linspace(0, 1, path_length), q_path(q_idx,:)));
t = linspace(0, 1, 10*path_length);
q_path = eval(q_traj, t);

% Determine max joint velocity at midpoint of  each segment
t_mid = mean([t(2:end); t(1:end-1)],1);
v_mid = max(abs(q_traj.fnder().eval(t_mid)), [], 1);

% Adjust durations to keep velocity below max
t_scaled = [0, cumsum(diff(t).*v_mid/mean(options.joint_v_max))];
tf = t_scaled(end);

% Warp time to give gradual acceleration/deceleration
t_scaled = tf*(-real(acos(2*t_scaled/tf-1)+pi)/2);
[t_scaled, idx_unique] = unique(t_scaled,'stable');

xtraj = PPTrajectory(pchip(t_scaled,[q_path(:,idx_unique); zeros(r.getNumVelocities(),numel(t_scaled))]));
xtraj = xtraj.setOutputFrame(r.getStateFrame());
end


