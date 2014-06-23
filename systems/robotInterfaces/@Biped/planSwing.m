function [swing_ts, swing_poses, takeoff_time, landing_time] = planSwing(biped, step1, step2)
% Compute a collision-free swing trajectory for a single foot.

assert(step1.frame_id == step2.frame_id, 'planSwing expects to plan a swing trajcectory between two positions of the /same/ foot body')

params = struct(step2.walking_params);
params = applyDefaults(params, biped.default_walking_params);
last_pos = step1.pos;
next_pos = step2.pos;

if params.step_speed < 0
  % negative step speed is an indicator to take a fast, fixed-duration step (e.g. for recovery)
  fixed_duration = -params.step_speed;
  params.step_speed = 1;
else
  fixed_duration = 0;
end

debug = false;

ignore_height = 0.5; % m, height above which we'll assume that our heightmap is giving us bad data (e.g. returns from an object the robot is carrying)

pre_contact_height = 0.005; % height above the ground to aim for when foot is landing
foot_yaw_rate = 0.75; % rad/s

step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));

if fixed_duration
  apex_fracs = [0.15,0.16, 0.84,0.85];
else
  apex_fracs = [0.15, 0.85];
end

next_pos(4:6) = last_pos(4:6) + angleDiff(last_pos(4:6), next_pos(4:6));
apex_pos = interp1([0, 1], [last_pos, next_pos]', apex_fracs)';
apex_pos(3,:) = last_pos(3) + params.step_height + max([next_pos(3) - last_pos(3), 0]);

apex_pos_l = [apex_fracs * step_dist_xy; apex_pos(3,:)];

terrain_pts = step2.terrain_pts;
if step_dist_xy > 0.01 && ~isempty(terrain_pts)
  if any(terrain_pts(2,:) > (max([last_pos(3), next_pos(3)]) + ignore_height))
    % If we're getting extremely high terrain heights, then assume it's bad lidar data
    terrain_pts(2,:) = max([last_pos(3), next_pos(3)]);
  end

  %% Expand terrain convex hull by the size of the foot
  expanded_terrain_pts = [[0;last_pos(3)], apex_pos_l, [step_dist_xy; next_pos(3) + pre_contact_height]];
  [contact_length, ~, contact_height] = contactVolume(biped, step1, step2, struct('nom_z_clearance', params.step_height));
  for j = 1:length(terrain_pts(1,:))
    if terrain_pts(2, j) > (j / length(terrain_pts(1,:))) * (next_pos(3) - last_pos(3)) + last_pos(3) + (params.step_height / 2)
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [-contact_length; contact_height];
      expanded_terrain_pts(:, end+1) = terrain_pts(:, j) + [contact_length; contact_height];
    end
  end
  expanded_terrain_pts(1,:) = bsxfun(@max, bsxfun(@min, expanded_terrain_pts(1,:), step_dist_xy), 0);
  expanded_terrain_pts = expanded_terrain_pts(:, convhull(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'simplify', true));
  expanded_terrain_pts = expanded_terrain_pts(:, end:-1:1); % convert counterclockwise to clockwise convex hull

  %% Draw our swing trajectory around the convex hull of the expanded terrain
  traj_pts = expanded_terrain_pts(:, 1:find(expanded_terrain_pts(1,:) >= step_dist_xy, 1, 'first'));

  % add start and end points
  traj_pts = [[0; last_pos(3)], traj_pts, [step_dist_xy; next_pos(3) + pre_contact_height]];
else
  %% Just ignore the terrain and provide an apex pose
  step_dist_xy = 1;
  traj_pts = [[0; last_pos(3)], [apex_fracs; apex_pos(3,:)], [1;next_pos(3) + pre_contact_height]];
end
traj_pts_xyz = [last_pos(1) + (next_pos(1) - last_pos(1)) * traj_pts(1,:) / step_dist_xy;
                last_pos(2) + (next_pos(2) - last_pos(2)) * traj_pts(1,:) / step_dist_xy;
                traj_pts(2,:)];


%% Compute time required for swing from cartesian distance of poses as well as yaw distance
d_dist = sqrt(sum(diff(traj_pts_xyz, 1, 2).^2, 1));
total_dist_traveled = sum(d_dist);
traj_dts = max([d_dist / params.step_speed;
                (d_dist / total_dist_traveled) .* (abs(angleDiff(next_pos(6), last_pos(6))) / foot_yaw_rate)],[],1);
traj_ts = [0, cumsum(traj_dts)] ;

%% Add time to shift weight
if fixed_duration
  hold_time = 0.1;
  traj_ts = traj_ts * ((fixed_duration - 2*hold_time) / traj_ts(end));
  traj_ts = [0, traj_ts+0.5*hold_time, traj_ts(end) + hold_time];
else
  hold_time = traj_ts(end) * params.hold_frac;
  hold_time = max([hold_time, params.drake_min_hold_time]);
  traj_ts = [0, traj_ts + 0.5 * hold_time, traj_ts(end) + hold_time]; % add time for weight shift
end

traj_pts_xyz = [last_pos(1:3), traj_pts_xyz, next_pos(1:3)];
landing_time = traj_ts(end-1);
takeoff_time = traj_ts(2);

%% Interpolate in rpy to constrain the foot orientation. We may set these values to NaN later to free up the foot orientation
rpy_pts = [last_pos(4:6), interp1(traj_ts([2,end-1]), [last_pos(4:6), next_pos(4:6)]', traj_ts(2:end-1))', next_pos(4:6)];

swing_poses.center = [traj_pts_xyz; rpy_pts];

swing_ts = traj_ts;


if debug
  figure(1)
  clf
  hold on
  plot(terrain_pts(1,:), terrain_pts(2,:), 'g.')
  plot(expanded_terrain_pts(1,:), expanded_terrain_pts(2,:), 'ro')

  t = linspace(traj_ts(1), traj_ts(end));
  xyz = step_traj.eval(t);
  plot(sqrt(sum(bsxfun(@minus, xyz(1:2,:), xyz(1:2,1)).^2)), xyz(3,:),'k')
  axis equal
end

end
