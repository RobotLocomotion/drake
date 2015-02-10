function [foot_origin_knots, zmp_knots] = planSwingPitched(biped, stance, swing1, swing2, initial_hold_time)
% Compute a collision-free swing trajectory for a single foot.
if nargin < 5
  initial_hold_time = 0;
end

assert(swing1.frame_id == swing2.frame_id, 'planSwing expects to plan a swing trajectory between two positions of the /same/ foot body')

params = struct(swing2.walking_params);
params = applyDefaults(params, biped.default_walking_params);

DEBUG = false;
DEFAULT_FOOT_PITCH = pi/8; % The amount by which the swing foot pitches forward during walking

APEX_FRACTIONS = [0.15, 0.92]; % We plan only two poses of the foot during the aerial phase of the swing.
                               % Those poses are planned for locations where the toe has traveled a given
                               % fraction of the distance from its initial location to its final location.

FOOT_YAW_RATE = 0.75; % rad/s
FOOT_PITCH_RATE = 2*pi; % rad/s

LATERAL_TOL = 1e-3; % Distance the sole can move to away from the line 
                    % between step1 and step2

MIN_DIST_FOR_TOE_OFF = 0.1; % minimum distance of *forward* progress for toe-off to be allowed.
                            % This disallows toe-off during backward stepping. 
                            % The distance is measured as the vector between the two swing poses
                            % dotted with the orientation of the stance foot

MAX_LANDING_SPEED = 0.75;
MAX_TAKEOFF_SPEED = 1.5;

if stance.frame_id == biped.foot_frame_id.right
  stance_foot_name = 'right';
else
  stance_foot_name = 'left';
end
if swing1.frame_id == biped.foot_frame_id.right
  swing_foot_name = 'right';
else
  swing_foot_name = 'left';
end

swing2.pos(4:6) = swing1.pos(4:6) + angleDiff(swing1.pos(4:6), swing2.pos(4:6));


xy_dist = norm(swing2.pos(1:2) - swing1.pos(1:2));

% The terrain slice is a 2xN matrix, where the first row is distance along the straight line path from swing1 to swing2 and the second row is height above the z position of swing1.
terrain_slice = double(swing2.terrain_pts);
terrain_slice = [[0;0], terrain_slice, [xy_dist; 0]];
terrain_pts_in_local = [terrain_slice(1,:); zeros(1, size(terrain_slice, 2)); 
                        terrain_slice(2,:)];

% Transform to world coordinates
T_local_to_world = [[rotmat(atan2(swing2.pos(2) - swing1.pos(2), swing2.pos(1) - swing1.pos(1))), [0;0];
                     0, 0, 1], [swing1.pos(1:2); 0]; 
                    0, 0, 0, 1];

% Determine how much of a forward step this is
swing_distance_in_local = (swing2.pos(1:2) - swing1.pos(1:2))' * (rotmat(stance.pos(6)) * [1;0]);

if swing_distance_in_local > MIN_DIST_FOR_TOE_OFF
  toe_off_angle = DEFAULT_FOOT_PITCH;
else
  toe_off_angle = 0;
end

% Create posture constraint
xstar = biped.loadFixedPoint();
xstar([1:2,6]) = stance.pos([1:2,6]);

joint_position_indices = (7:biped.getNumPositions())';
posture_constraint = PostureConstraint(biped);
q_joints = xstar(joint_position_indices);
posture_constraint = posture_constraint.setJointLimits(joint_position_indices, q_joints, q_joints);

swing_body_index = biped.getFrame(swing1.frame_id).body_ind;
stance_body_index = biped.getFrame(stance.frame_id).body_ind;
swing_toe_points_in_foot = biped.getBody(swing_body_index).getTerrainContactPoints('toe');
T_sole_to_foot = biped.getFrame(swing1.frame_id).T;

T_swing1_sole_to_world = ...
  [rpy2rotmat(swing1.pos(4:6)),swing1.pos(1:3); zeros(1, 3), 1];
T_swing1_foot_to_world = T_swing1_sole_to_world/T_sole_to_foot;
swing1_toe_points_in_world = T_swing1_foot_to_world * ...
  [swing_toe_points_in_foot; ones(1,size(swing_toe_points_in_foot,2))];

T_swing2_sole_to_world = ...
  [rpy2rotmat(swing2.pos(4:6)),swing2.pos(1:3); zeros(1, 3), 1];
T_swing2_foot_to_world = T_swing2_sole_to_world/T_sole_to_foot;
swing2_toe_points_in_world = T_swing2_foot_to_world * ...
  [swing_toe_points_in_foot; ones(1,size(swing_toe_points_in_foot,2))];

toe1 = mean(swing1_toe_points_in_world(1:3,:), 2);
toe2= mean(swing2_toe_points_in_world(1:3,:), 2);
T_toe_local_to_world = [[rotmat(atan2(toe2(2) - toe1(2), toe2(1) - toe1(1))), [0;0];
                     0, 0, 1], toe1(1:3); 
                    0, 0, 0, 1];
terrain_pts_in_toe_local = inv(T_toe_local_to_world) * T_local_to_world * [terrain_pts_in_local; ones(1, size(terrain_pts_in_local,2))];

swing1_toe_points_in_toe_local = T_toe_local_to_world \ swing1_toe_points_in_world;
swing2_toe_points_in_toe_local = T_toe_local_to_world \ swing2_toe_points_in_world;

quat_toe_off = rotmat2quat(rpy2rotmat(swing1.pos(4:6)) * rpy2rotmat([0;toe_off_angle;0]) / T_sole_to_foot(1:3,1:3));
quat_swing2 = rotmat2quat(rpy2rotmat(swing2.pos(4:6)) / T_sole_to_foot(1:3,1:3));

cost = Point(biped.getStateFrame(),1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_roll = 0;
cost.base_pitch = 0;
cost.base_yaw = 0;
ikoptions = IKoptions(biped);
ikoptions = ikoptions.setQ(diag(cost(1:biped.getNumPositions())));

q_latest = xstar(1:biped.getNumPositions());

if DEBUG
  v = biped.constructVisualizer();
  v.draw(0, q_latest);
end

quat_tol = 1e-6;

T = biped.getFrame(stance.frame_id).T;
stance_sole = [rpy2rotmat(stance.pos(4:6)), stance.pos(1:3); 0 0 0 1];
stance_origin = stance_sole / T;
stance_origin_pose = [stance_origin(1:3,4); rotmat2rpy(stance_origin(1:3,1:3))];

T = biped.getFrame(swing1.frame_id).T;
swing1_sole = [rpy2rotmat(swing1.pos(4:6)), swing1.pos(1:3); 0 0 0 1];
swing1_origin = swing1_sole / T;
swing1_origin_pose = [swing1_origin(1:3,4); rotmat2rpy(swing1_origin(1:3,1:3))];

T = biped.getFrame(swing2.frame_id).T;
swing2_sole = [rpy2rotmat(swing2.pos(4:6)), swing2.pos(1:3); 0 0 0 1];
swing2_origin = swing2_sole / T;
swing2_origin_pose = [swing2_origin(1:3,4); rotmat2rpy(swing2_origin(1:3,1:3))];

instep_shift = [0.0;stance.walking_params.drake_instep_shift;0];
zmp1 = shift_step_inward(biped, stance, instep_shift);

hold_time = params.drake_min_hold_time;
zmp_knots = struct('t', initial_hold_time + (hold_time / 2),...
 'zmp', zmp1, ...
 'supp', RigidBodySupportState(biped, stance_body_index));

foot_origin_knots = struct('t', zmp_knots(end).t, ...
                           swing_foot_name, [swing1_origin_pose; zeros(6,1)], ...
                           stance_foot_name, [stance_origin_pose; zeros(6,1)], ...
                           'is_liftoff', true,...
                           'is_landing', false,...
                           'toe_off_allowed', struct(swing_foot_name, swing_distance_in_local >= MIN_DIST_FOR_TOE_OFF, stance_foot_name, false));

function [pose, q0] = solve_for_pose(constraints, q0)
  constraint_ptrs = {};
  for k = 1:length(constraints)
    constraint_ptrs{end+1} = constraints{k}.mex_ptr;
  end
  [q0, info] = inverseKin(biped,q0,q0,constraint_ptrs{:},ikoptions);
  if info >= 10
    % try again?
    [q0, info] = inverseKin(biped,q0,q0,constraint_ptrs{:},ikoptions);
    if info < 10
      warning('Whoa...tried inverseKin again and got a different result...');
    else
      error('Drake:planSwingPitched', 'The foot pose IK problem could not be solved. This should not happen and likely indicates a bug in the constraints.');
    end
  end
  if DEBUG
    v.draw(0, q0);
  end
  kinsol = biped.doKinematics(q0);
  pose = biped.forwardKin(kinsol, swing_body_index, [0;0;0], 1);
end

function add_foot_origin_knot(swing_pose, speed)
  if nargin < 2
    speed = params.step_speed;
  end
  foot_origin_knots(end+1).(swing_foot_name) = [swing_pose; zeros(6,1)];
  foot_origin_knots(end).(stance_foot_name) = [stance_origin_pose; zeros(6,1)];
  cartesian_distance = norm(foot_origin_knots(end).(swing_foot_name)(1:3) - foot_origin_knots(end-1).(swing_foot_name)(1:3));
  sole1 = rotmat2rpy(rpy2rotmat(foot_origin_knots(end-1).(swing_foot_name)(4:6)) * T_sole_to_foot(1:3,1:3));
  sole2 = rotmat2rpy(rpy2rotmat(foot_origin_knots(end).(swing_foot_name)(4:6)) * T_sole_to_foot(1:3,1:3));
  yaw_distance = abs(angleDiff(sole1(3), sole2(3)));
  pitch_distance = abs(angleDiff(sole1(2), sole2(2)));
  dt = max([cartesian_distance / speed,...
            yaw_distance / FOOT_YAW_RATE,...
            pitch_distance / FOOT_PITCH_RATE]);
  foot_origin_knots(end).t = foot_origin_knots(end-1).t + dt;
  foot_origin_knots(end).is_liftoff = false;
  foot_origin_knots(end).is_landing = false;
  foot_origin_knots(end).toe_off_allowed = struct(swing_foot_name, false, stance_foot_name, false);
end

% Apex knot 1
max_terrain_ht = max(terrain_pts_in_toe_local(3,:));
toe_ht_in_local = max_terrain_ht + params.step_height;
toe_pos_in_local = interp1([0, 1], [mean(swing1_toe_points_in_toe_local, 2), mean(swing2_toe_points_in_toe_local, 2)]', APEX_FRACTIONS(1))';
constraints = {posture_constraint,...
               WorldQuatConstraint(biped, swing_body_index, quat_toe_off, quat_tol),...
               WorldPositionInFrameConstraint(biped,swing_body_index,...
                    mean(swing_toe_points_in_foot, 2), T_toe_local_to_world, [toe_pos_in_local(1); -LATERAL_TOL; toe_ht_in_local], [toe_pos_in_local(1); LATERAL_TOL; toe_ht_in_local])};
[pose, q_latest] = solve_for_pose(constraints, q_latest);
add_foot_origin_knot(pose, min(params.step_speed, MAX_TAKEOFF_SPEED)/2);

% Apex knot 2
max_terrain_ht = max(terrain_pts_in_toe_local(3,:));
toe_ht_in_local = max_terrain_ht + params.step_height;
toe_pos_in_local = interp1([0, 1], [mean(swing1_toe_points_in_toe_local, 2), mean(swing2_toe_points_in_toe_local, 2)]', APEX_FRACTIONS(2))';
constraints = {posture_constraint,...
               WorldQuatConstraint(biped, swing_body_index, quat_swing2, quat_tol),...
               WorldPositionInFrameConstraint(biped,swing_body_index,...
                    mean(swing_toe_points_in_foot, 2), T_toe_local_to_world, [toe_pos_in_local(1); -LATERAL_TOL; toe_ht_in_local], [toe_pos_in_local(1); LATERAL_TOL; toe_ht_in_local])};
[pose, ~] = solve_for_pose(constraints, q_latest);
add_foot_origin_knot(pose);

% Set the target velocities of the two apex poses based on the total distance traveled
foot_origin_knots(end).(swing_foot_name)(7:9) = (foot_origin_knots(end).(swing_foot_name)(1:3) - foot_origin_knots(end-1).(swing_foot_name)(1:3)) / (foot_origin_knots(end).t - foot_origin_knots(end-1).t);
foot_origin_knots(end-1).(swing_foot_name)(7:9) = (foot_origin_knots(end).(swing_foot_name)(1:3) - foot_origin_knots(end-1).(swing_foot_name)(1:3)) / (foot_origin_knots(end).t - foot_origin_knots(end-1).t);
% angles require unwrapping to get the correct velocities
foot_origin_knots(end).(swing_foot_name)(10:12) = angleDiff(foot_origin_knots(end-1).(swing_foot_name)(4:6), foot_origin_knots(end).(swing_foot_name)(4:6)) / (foot_origin_knots(end).t - foot_origin_knots(end-1).t);
foot_origin_knots(end-1).(swing_foot_name)(10:12) = angleDiff(foot_origin_knots(end-1).(swing_foot_name)(4:6), foot_origin_knots(end).(swing_foot_name)(4:6)) / (foot_origin_knots(end).t - foot_origin_knots(end-1).t);


% Landing knot
add_foot_origin_knot(swing2_origin_pose, min(params.step_speed, MAX_LANDING_SPEED)/2);
foot_origin_knots(end).is_landing = true;
zmp_knots(end+1).t = foot_origin_knots(end).t;
zmp_knots(end).zmp = zmp1;
zmp_knots(end).supp = RigidBodySupportState(biped, [stance_body_index, swing_body_index]);

% Final knot
foot_origin_knots(end+1) = foot_origin_knots(end);
foot_origin_knots(end).t = foot_origin_knots(end-1).t + hold_time / 2;
end

function pos = shift_step_inward(biped, step, instep_shift)
  if step.frame_id == biped.foot_frame_id.left
    instep_shift = [1;-1;1].*instep_shift;
  end
  pos_center = step.pos;
  R = rpy2rotmat(pos_center(4:6));
  shift = R*instep_shift;
  pos = pos_center(1:2) + shift(1:2);
end

