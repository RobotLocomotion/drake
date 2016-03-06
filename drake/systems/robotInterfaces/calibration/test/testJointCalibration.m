function testJointCalibration
testJointOffsetCalibration();
testJointStiffnessCalibration();
end

function testJointOffsetCalibration()
num_poses = 10;
[r, q_actual, q_indices, bodies, marker_positions_actual, marker_functions, num_unmeasured_markers, vicon_data] = setUp(num_poses);

q_offset_max = 1e-2;
q_noise_stddev = 1e-6;

s = rng(1234, 'twister');
q_offset_actual = 2 * (rand(length(q_indices), 1) - 0.5) * q_offset_max;
q_measured = q_actual + q_noise_stddev * randn(size(q_actual));
q_measured(q_indices, :) = q_measured(q_indices, :) - repmat(q_offset_actual, 1, num_poses);
scales = {100, 1};
rng(s);

options.search_floating = true;
[q_offset_estimated, marker_params, floating_states] = jointOffsetCalibration(r, q_measured, q_indices, ...
  bodies, marker_functions, cellfun(@(x) 3 * x, num_unmeasured_markers, 'UniformOutput', false), vicon_data, scales, options);

q_offset_error = angleDiff(q_offset_actual, q_offset_estimated);

fprintf('q_offset_error:\n');
disp(q_offset_error);

valuecheck(q_offset_error, zeros(size(q_offset_actual)), 1e-2);
checkMarkerPositions(marker_positions_actual, marker_functions, marker_params);
checkFloatingStates(r, bodies, q_actual, floating_states, num_poses);

end

function testJointStiffnessCalibration()
num_poses = 30;
k_nominal = 0.5e4;
k_max_deviation = 1e4;
k_initial_stddev = 1e3;
q_noise_stddev = 1e-5;
u_stddev = 150;

[r, q_actual, q_indices, bodies, marker_positions_actual, marker_functions, num_unmeasured_markers, vicon_data] = setUp(num_poses);

s = rng(1234, 'twister');

nk = length(q_indices);
k_actual = k_nominal * ones(nk, 1) + rand(nk, 1) * k_max_deviation;
u_data = u_stddev * randn(r.getNumInputs(), num_poses);

B = r.getB();
[rows, u_indices] = find(B(q_indices, :) ~= 0);
[~, sort_indices] = sort(rows);
u_indices = u_indices(sort_indices);
dq_actual = diag(1 ./ k_actual) * B(q_indices, u_indices) * u_data(u_indices, :);

q_measured = q_actual + q_noise_stddev * randn(size(q_actual));
q_measured(q_indices, :) = q_measured(q_indices, :) - dq_actual;

k_initial = abs(k_actual + k_initial_stddev * randn(nk, 1));
scales = {1, 1};

rng(s);

options.search_floating = true;

[k_estimated, marker_params, floating_states] = jointStiffnessCalibration(r, q_measured, u_data, q_indices, ...
  bodies, marker_functions, cellfun(@(x) 3 * x, num_unmeasured_markers, 'UniformOutput', false), vicon_data, ...
  scales, k_initial, options);

fprintf('k_error:\n');
disp(k_estimated - k_actual);

valuecheck(k_estimated, k_actual, k_nominal / 20);
checkMarkerPositions(marker_positions_actual, marker_functions, marker_params);
checkFloatingStates(r, bodies, q_actual, floating_states, num_poses);

end

function [r, q_actual, q_indices, bodies, marker_positions_actual, marker_functions, num_unmeasured_markers, vicon_data] = setUp(num_poses)
options.floating = 'rpy';
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
r = RigidBodyManipulator(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),options);
warning(w);

bodies{1} = r.findLinkId('utorso');
bodies{2} = r.findLinkId('r_hand');
[~, joint_indices] = r.findKinematicPath(bodies{1}, bodies{2});
q_indices = [r.getBody(joint_indices).position_num];

% settings
marker_measurement_stddev = 1e-5;
vicon_data_noise_stddev = 1e-6;
marker_offset_max = 0.1;
num_markers = 4;
num_measured_markers = 3;

s = rng(215615, 'twister');

% actual ('perfect') joint configurations
q_actual = zeros(r.getNumPositions(), num_poses);
for i = 1 : num_poses
  q_actual(:, i) = getRandomConfiguration(r);
end
% make sure pitch is not close to +/- pi / 2
epsilon = 0.1;
for i = 1 : num_poses
  while abs(abs(q_actual(5, i)) - pi / 2) < epsilon
    q_actual(5, i) = randn;
  end
end

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

marker_functions = cell(length(bodies), 1);
for i = 1 : length(bodies)
  marker_functions{i} = @(params) markerPositions(params, marker_positions_measured{i});
end

rng(s);
end

function [x, dx] = markerPositions(params, marker_positions_measured)
x = marker_positions_measured;
measured_markers = ~any(isnan(marker_positions_measured), 1);
x(:, ~measured_markers) = reshape(params, 3, sum(~measured_markers));
dx = zeros(numel(x), numel(params));
dx = setSubMatrixGradient(dx, eye(numel(params)), 1:3, find(~measured_markers), size(x));
end

function checkMarkerPositions(marker_positions_actual, marker_functions, body_params)
for i = 1 : length(marker_positions_actual)  
  % marker positions
  valuecheck(marker_functions{i}(body_params{i}), marker_positions_actual{i}, 1e-2);
end
end

function checkFloatingStates(r, bodies, q_actual, floating_states, num_poses)
for i = 1 : length(bodies)
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