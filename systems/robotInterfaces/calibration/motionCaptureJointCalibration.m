function [q_correction_params, marker_params, floating_states, objective_value, marker_residuals, info] = motionCaptureJointCalibration(...
  p, q_correction_fun, q_data, joint_indices,...
  bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, options)
% Perform joint calibration, given motion capture data and joint data.
% Given (x,y,z) position data of some set of markers on various
% bodies and nominal joint angles, attemps to fit three sets of parameters:
%  (1) Joint correction parameters that can be used in conjunction with a
%  correction function to obtain a better estimate of the true joint angles
%  (2) Parameters for the locations of the markers on each of the bodies
%  (3) The floating base state of each sample in time
% 
% @param p Robot plant
% @param q_correction_fun a function 
% [q_data_mod, dq_data_mod] = f(q_data(joint_indices, :), params)
% where q_mod is a modified version of q_data(joint_indices) and params
% (length(joint_indices) x 1) are unknown parameters
% @param q_data (nxN) joint data
% @param joint_indices joint configuration vector indices corresponding to
% joints to calibrate
% @param bodies nb x 1 cell array of body indices to which motion capture markers
% were attached
% @param marker_functions nb x 1 cell array of functions [x,dx]=f(p) where 
% x (3 x m_i) is the position of the markers in the bodies{i} frame, 
% p (marker_function_num_params{i} x 1) are unknown parameters
% @param marker_function_num_params nb x 1 cell array where
% marker_function_num_params{i} is the number of parameters describing the  
% positions of the markers on bodies{i}
% @param motion_capture_data nb x 1 cell array of motion capture position
% data. motion_capture_data{i} is a 3 x m_i x N array containing the
% measured positions of the markers attached to bodies{i}: the columns
% of motion_capture_data{i}(:, :, j) are the individual measured marker
% positions. NaNs indicate occluded markers.
% @param scales nb x 1 cell array of scalings used to weight errors in
% marker positions per body
% @param options option struct: 
%   options.search_floating: logical indicating whether to find floating
%   states given motion capture data
%   options.initial_guess: initial guess for q_correction_params
%
% @retval q_correction_params parameters found for use with q_correction_fun
% @retval marker_params nb x 1 cell array where marker_params{i} contains
% the optimized parameters found for marker_functions{i}
% @retval floating_states (6 x N) floating base states found for the robot
% @retval objective_value objective value at solution
% @retval marker_residuals nb x 1 cell array where marker_residuals{i}
% contains the measurement residuals of the markers attached to bodies{i}
% @retval info info as returned by fminunc

njoints = length(joint_indices);
nposes = size(q_data,2);
nbodies = length(bodies);

if nargin < 10
  options = struct();
end
if ~isfield(options,'search_floating')
  options.search_floating = true;
end
if ~isfield(options,'initial_guess')
  options.initial_guess = zeros(njoints, 1);
end

marker_function_num_params = cell2mat(marker_function_num_params);
if options.search_floating
  floating_body = p.getBody(2);
  if floating_body.floating ~= 1
    error('Calling this function with options.search_floating = true currently requires the first joint in the kinematic chain to be an rpy-parameterized floating joint.');
  end
  floating_indices = floating_body.position_num;
  floating_states0 = zeros(7, nposes);
  floating_states0(1 : 3, :) = q_data(floating_body.position_num(1 : 3), :);
  for i = 1 : nposes
    floating_states0(4 : 7, i) = rpy2quat(q_data(floating_body.position_num(4 : 6), i));
  end  
else
  floating_indices = zeros(0, 1);
  floating_states0 = zeros(0, nposes);
end

f = @(params) markerResiduals(p, q_correction_fun, q_data, joint_indices, floating_indices,...
  bodies, marker_functions, motion_capture_data, scales, ...
  params(1:njoints), mat2cell(params(njoints + 1 : njoints + sum(marker_function_num_params)), marker_function_num_params), reshape(params(end - numel(floating_states0) + 1 : end), size(floating_states0)));

fmin_options = optimset('Display','iter-detailed','GradObj','on'); %, 'DerivativeCheck','on','FinDiffType','central');

x0 = [options.initial_guess; zeros(sum(marker_function_num_params),1); floating_states0(:)];
[X,FVAL,EXITFLAG] = fminunc(f,x0,fmin_options);

q_correction_params = X(1:njoints);
marker_params = cell(nbodies, 1);
marker_param_start_row = njoints + 1;
for i = 1 : nbodies
  marker_params{i} = X(marker_param_start_row : marker_param_start_row + marker_function_num_params(i) - 1);
  marker_param_start_row = marker_param_start_row + marker_function_num_params(i);
end

if options.search_floating
  % transform back from quaternion parameterization to rpy
  floating_params = reshape(X(end - numel(floating_states0) + 1 : end), size(floating_states0)); % [pos; quat] for
  pos_rows = 1:3;
  quat_rows = 4:7;
  floating_states = zeros(length(floating_indices), nposes);
  for i = 1 : nposes
    pos = floating_params(pos_rows, i);
    R = quat2rotmat(floating_params(quat_rows, i)); % includes normalization
    rpy = rotmat2rpy(R);
    floating_states(:, i) = [pos; rpy];
  end
else
  floating_states = floating_states0;
end;

objective_value = FVAL;
info = EXITFLAG;

[~,~,marker_residuals] = f(X);
end

function [f, g, marker_residuals] = markerResiduals(...
  p, q_correction_fun, q_data, joint_indices, floating_indices,...
  bodies, marker_functions, motion_capture_data, scales, ...
  q_correction_params, marker_params, floating_params)
  
nbodies = length(bodies);
nq = size(q_data, 1);
nposes = size(q_data, 2);

[q_data(joint_indices, :), dqdq_correction_params] = q_correction_fun(q_data(joint_indices, :), q_correction_params);

% floating states are parameterized as floating_states(:, i) = [pos; quat];
% need to normalize quaternion first:
if ~isempty(floating_params)
  q_floating_size = [length(floating_indices) nposes];
  dq_floatingdfloating_params = sparse(prod(q_floating_size), numel(floating_params));

  pos_rows = 1:3;
  quat_rows = 4:7;
  rpy_rows = 4:6;
  for i = 1 : nposes
    pos = floating_params(pos_rows, i);
    [R, dRdquat] = quat2rotmat(floating_params(quat_rows, i)); % includes normalization
    [rpy, drpydquat] = rotmat2rpy(R, dRdquat);
    q_data(floating_indices, i) = [pos; rpy];
    
    pos_indices = sub2ind(size(floating_params), pos_rows, repmat(i, 1, length(pos_rows)));
    quat_indices = sub2ind(size(floating_params), quat_rows, repmat(i, 1, length(quat_rows)));
    dq_floatingdfloating_params = setSubMatrixGradient(dq_floatingdfloating_params, eye(3), pos_rows, i, q_floating_size, pos_indices);
    dq_floatingdfloating_params = setSubMatrixGradient(dq_floatingdfloating_params, drpydquat, rpy_rows, i, q_floating_size, quat_indices);
  end  
end

f = 0;
dfdq_correction_params = zeros(1, numel(q_correction_params));
dfdbody_params = cell(nbodies, 1);
dfdfloating_params = zeros(1, numel(floating_params));

if nargout > 2
  marker_residuals = cell(nbodies, 1);
end

for i = 1 : nbodies 
  [pts, dpts] = marker_functions{i}(marker_params{i});
  x_body = zeros(size(pts, 1), size(pts, 2), nposes);
  J_body = zeros(numel(pts), nq, nposes);
  R_body = cell(nposes, 1);
  
  for j = 1 : nposes
    kinsol = p.doKinematics(q_data(:,j));
    [x_body(:, :, j), J_body(:, :, j)] = p.forwardKin(kinsol, bodies{i}, pts);
    
    % awkward hack to get out the rotation matrix
    R_body{j} = p.forwardKin(kinsol, bodies{i}, eye(3)) - p.forwardKin(kinsol, bodies{i}, zeros(3));
  end

  % remove obscured markers from calculation (nan)
  % this will also zero out any gradient effects, since cost is purely quadratic
  % df/dy * (x_body - body_data);
  nan_indices = isnan(motion_capture_data{i});
  motion_capture_data{i}(nan_indices) = x_body(nan_indices);
  
  
  body_err = x_body(:) - motion_capture_data{i}(:);
  f = f + scales{i} * (body_err' * body_err);
  
  % Gradient computation
  % dfdq_correction_params
  J_body_joint_indices_cell = mat2cell(J_body(:, joint_indices, :), size(J_body, 1), length(joint_indices), ones(1, nposes));
  dbody_errdq_correction_params = blkdiag(J_body_joint_indices_cell{:}) * dqdq_correction_params;
  dfdq_correction_params = dfdq_correction_params + scales{i} * 2 * body_err' * dbody_errdq_correction_params;
  
  % dfdbody_params
  dbody_err_dbody_params = cellMatGradMultMat(R_body, pts, dpts);
  dfdbody_params{i} = scales{i} * 2 * body_err' * dbody_err_dbody_params;
  
  % dfdfloating_params
  if ~isempty(floating_params)
    dbody_errdfloating_states_diag = mat2cell(J_body(:, floating_indices, :), size(J_body, 1), length(floating_indices), ones(1, nposes));
    dbody_errdfloating_params = blkdiag(dbody_errdfloating_states_diag{:}) * dq_floatingdfloating_params;
    dfdfloating_params = dfdfloating_params + scales{i} * 2 * body_err' * dbody_errdfloating_params;
  end
  
  if nargout > 2
    marker_residuals{i} = x_body - motion_capture_data{i};
  end
end

g = [dfdq_correction_params dfdbody_params{:} dfdfloating_params];



% TODO: update these
% Evaluation outputs
% if nargout > 2
%   J_qoffset = zeros(n_joints,(m1+m2)*N*3);
%   J_body1_params = zeros(body1_num_params,(m1+m2)*N*3);
%   J_body2_params = zeros(body2_num_params,(m1+m2)*N*3);
%   J_floating_states = zeros(6,N,(m1+m2)*N*3);
%   J = zeros(body1_num_params+body2_num_params+n_joints+6*N,(m1+m2)*N*3);
%   for i=1:N,
%     for j=1:m1,
%       J_qoffset(:,(i-1)*m1*3 + (j-1)*3 + (1:3)) = J_body1((1:3) + (j-1)*3,joint_indices,i)';
%       J_body1_params(:,(i-1)*m1*3 + (j-1)*3 + (1:3)) = dpts_body1((1:3) + (j-1)*3,:)'*R_body1{i}';
%       J_floating_states(:,i,(i-1)*m1*3 + (j-1)*3 + (1:3)) = J_body1((1:3) + (j-1)*3,1:6,i)';
%     end
%     
%     for j=1:m2,
%       J_qoffset(:,N*m1*3 + (i-1)*m2*3 + (j-1)*3 + (1:3)) = J_body2((1:3) + (j-1)*3,joint_indices,i)';
%       J_body2_params(:,N*m1*3 + (i-1)*m2*3 + (j-1)*3 + (1:3)) = dpts_body2((1:3) + (j-1)*3,:)'*R_body2{i}';
%       J_floating_states(:,i,N*m1*3 + (i-1)*m2*3 + (j-1)*3 + (1:3)) = J_body2((1:3) + (j-1)*3,1:6,i)';
%     end
%   end
%   if ~isempty(floating_states)
%     J = [J_qoffset; J_body1_params; J_body2_params; reshape(J_floating_states,[],(m1+m2)*N*3)];
%   else
%     J = [J_qoffset; J_body1_params; J_body2_params];
%   end
% end
% 
end