function testJointOffsetCalibration
options.floating = 'rpy';
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
r = RigidBodyManipulator(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),options);
warning(w);

bodies{1} = r.findLinkInd('utorso');
bodies{2} = r.findLinkInd('r_hand');
[~, joint_indices] = r.findKinematicPath(bodies{1}, bodies{2});

% settings
num_poses = 25;
marker_measurement_stddev = 1e-4;
vicon_data_noise_stddev = 1e-5;
q_noise_stddev = 1e-6;
marker_offset_max = 0.1;
q_offset_max = 1e-2;
num_markers = 5;
num_measured_markers = 3;

% actual ('perfect') joint configurations
nq = r.getNumPositions();
q_actual = randn(nq, num_poses); % todo: use getRandomConfiguration

% marker positions
marker_positions_actual = cell(length(bodies), 1);
num_unmeasured_markers = cell(length(bodies), 1);
marker_positions_measured = cell(length(bodies), 1);
for i = 1 : length(bodies)
%   marker_positions_actual{i} = 2 * (rand(3, num_markers) - 0.5) * marker_offset_max;
%   marker_positions_measured{i} = nan(size(marker_positions_actual{i}));
%   measured_markers = unique(randi(num_markers, 1, max_measured_markers));
%   measurement_error = marker_measurement_stddev * randn(3, length(measured_markers));
%   marker_positions_measured{i}(:, measured_markers) = marker_positions_actual{i}(:, measured_markers) + measurement_error;
%   num_unmeasured_markers{i} = num_markers - length(measured_markers);

  marker_positions_actual{i} = 2 * (rand(3, num_markers) - 0.5) * marker_offset_max;
  marker_positions_measured{i} = nan(size(marker_positions_actual{i}));
  measured_markers = randperm(num_markers, num_measured_markers);
  measurement_error = marker_measurement_stddev * randn(3, length(measured_markers));
  marker_positions_measured{i}(:, measured_markers) = marker_positions_actual{i}(:, measured_markers) + measurement_error;
  num_unmeasured_markers{i} = num_markers - length(measured_markers);
end

% simulated Vicon data
vicon_data = cell(length(bodies), 1);
for i = 1 : num_poses
  q = q_actual(:, i);
  kinsol = r.doKinematics(q, false, true);
  for j = 1 : length(bodies)
    body = bodies{j};
    marker_positions_world = r.forwardKin(kinsol, body, marker_positions_actual{j});
    vicon_noise = vicon_data_noise_stddev * randn(size(marker_positions_world));
    vicon_data{j}(:, :, i) = marker_positions_world + vicon_noise;
  end
end

q_indices = [r.getBody(joint_indices).position_num];
q_offset_actual = 2 * (rand(length(q_indices), 1) - 0.5) * q_offset_max;
q_measured = q_actual + q_noise_stddev * randn(size(q_actual));
q_measured(q_indices, :) = q_measured(q_indices, :) - repmat(q_offset_actual, 1, num_poses);
  
markerFunctions = cell(length(bodies), 1);
for i = 1 : length(bodies)
  markerFunctions{i} = @(params) markerPositions(params, marker_positions_measured{i});
end

options.search_floating = true;
[q_offset_estimated, body1_params, body2_params, floating_states] = jointOffsetCalibration(r, q_measured, q_indices, ...
  bodies{1}, markerFunctions{1}, num_unmeasured_markers{1} * 3, vicon_data{1}, ...
  bodies{2}, markerFunctions{2}, num_unmeasured_markers{2} * 3, vicon_data{2}, options);

body_params{1} = body1_params;
body_params{2} = body2_params;

% offsets
q_offset_error = angleDiff(q_offset_actual, q_offset_estimated);
fprintf('q_offset_error:\n');
disp(q_offset_error);
valuecheck(q_offset_error, zeros(size(q_offset_actual)), 1e-2);

for i = 1 : length(bodies)  
  % marker positions
  valuecheck(markerFunctions{i}(body_params{i}), marker_positions_actual{i}, 1e-2);
  
  % floating states
  for j = 1 : num_poses
    q_floating_actual = q_actual(r.getBody(2).position_num, j);
    q_floating_estimated = floating_states(:, j);
    T_actual = jointTransform(r.getBody(2), q_floating_actual);
    T_estimated = jointTransform(r.getBody(2), q_floating_estimated);
    T_diff = T_actual \ T_estimated;
    axis_angle = rotmat2axis(T_diff(1:3, 1:3));
    valuecheck(axis_angle(end), 0, 1e-2);
    valuecheck(T_diff(1:3, 4), zeros(3, 1), 1e-2);
  end
end

end

function [x, dx] = markerPositions(params, marker_positions_measured)
x = marker_positions_measured;
measured_markers = ~any(isnan(marker_positions_measured), 1);
x(:, ~measured_markers) = reshape(params, 3, sum(~measured_markers));
dx = zeros(numel(x), numel(params));
dx = setSubMatrixGradient(dx, eye(numel(params)), 1:3, find(~measured_markers), size(x));
end
