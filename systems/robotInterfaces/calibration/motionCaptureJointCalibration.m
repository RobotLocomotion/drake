function [q_correction_params, marker_params, floating_states, objective_value, marker_residuals, info] = motionCaptureJointCalibration(...
  p, q_correction_fun, q_data, joint_indices,...
  bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, options)
% Perform joint calibration, given vicon and joint data.
% Given (x,y,z) position data of some set of markers on two different
% bodies and nominal joint angles, attemps to fit three sets of parameters:
%  (1) Joint correction parameters that can be used in conjunction with a
%  correction function to obtain a better estimate of the true joint angles
%  (2) Parameters for the locations of the markers on both bodies
%  (3) The floating base state of each sample in time
% 
% @param p Robot plant
% @param q_correction_fun a function [q_data_mod, dq_data_mod] = f(q_data, params)
% where q_mod is a modified version of q(joint_indices) and params
% (length(joint_indices) x 1) are unknown parameters
% @param q_data (nxN) joint data
% @param joint_indices indices of joints to calibrate
% @param body1
% @param body1_marker_fun a function [x,dx]=f(p) where x (3xm1) is the position of
% the markers in the body1 frame, p (body1_num_paramsx1) are unknown parameters
% (body1_num_paramsx1) are unknown parameters
% @param body1_num_params number of parameters describing the positions of
% the markers on body1
% @param body2_data (3xm1xN) position data from the body2 markers. NaNs
% should be used to indicate occluded markers
% @param body2
% @param body2_marker_fun a function [x,dx]=f(p) where x (3xm2) is the position of
% the markers in the body2 frame, p (body2_num_paramsx1) are unknown parameters
% @param body2_num_params number of parameters describing the positions of
% the markers on body2
% @param body2_data (3xm2xN) position data from the body2 markers. NaNs
% should be used to indicate occluded markers
%
% @return q_correction_params parameters found for use with q_correction_fun
% @return body1_params parameters found for body1_marker_fun
% @return body2_params parameters found for body2_marker_fun
% @return floating_states (6xN) Floating base states found for the robot
% @return residuals Residual cost
% @return info As returned by fminunc
% @return J A jacobian

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

% zero out all other joints
% q_data(setdiff(1:n,joint_indices),:) = 0*q_data(setdiff(1:n,joint_indices),:);

marker_function_num_params = cell2mat(marker_function_num_params);
if options.search_floating
  floating_joint_nq = 6; %TODO
else
  floating_joint_nq = 0;
end

f = @(params) markerResiduals(p, q_correction_fun, q_data, joint_indices, ...
  bodies, marker_functions, motion_capture_data, scales, ...
  params(1:njoints), mat2cell(params(njoints + 1 : njoints + sum(marker_function_num_params)), marker_function_num_params), reshape(params(end - floating_joint_nq * nposes + 1 : end), [floating_joint_nq nposes]));

fmin_options = optimset('Display','iter-detailed','GradObj','on'); %,'DerivativeCheck','on','FinDiffType','central');

% p0 = randn(n_joints+6*N+body1_num_params + body2_num_params,1);
% [x,dx] = f2(p0);
% [x2,dx2] = fg(p0);
% keyboard
x0 = [options.initial_guess; zeros(sum(marker_function_num_params) + floating_joint_nq * nposes,1)];
[X,FVAL,EXITFLAG] = fminunc(f,x0,fmin_options);

q_correction_params = X(1:njoints);
marker_params = cell(nbodies, 1);
marker_param_start_row = njoints + 1;
for i = 1 : nbodies
  marker_params{i} = X(marker_param_start_row : marker_param_start_row + marker_function_num_params(i) - 1);
  marker_param_start_row = marker_param_start_row + marker_function_num_params(i);
end

floating_states = reshape(X(end - floating_joint_nq * nposes + 1 : end), [floating_joint_nq nposes]);
objective_value = FVAL;
info = EXITFLAG;

[~,~,marker_residuals] = f(X);
end

function [f, g, marker_residuals] = markerResiduals(p, q_correction_fun, q_data, joint_indices, ...
  bodies, marker_functions, motion_capture_data, scales, ...
  q_correction_params, marker_params, floating_states)
  
[q_data(joint_indices, :), dqdq_correction_params] = q_correction_fun(q_data(joint_indices, :), q_correction_params);

floating_indices = 1:6; % TODO: generalize

if ~isempty(floating_states)
  q_data(floating_indices,:) = floating_states;
end

nbodies = length(bodies);
nq = size(q_data, 1);
nposes = size(q_data, 2);

f = 0;
dfdq_correction_params = zeros(1, numel(q_correction_params));
dfdbody_params = cell(nbodies, 1);
dfdfloating_states = zeros(1, numel(floating_states));

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
  
  % dfdfloating_states
  if ~isempty(floating_states)
    dbody_errdfloating_states_diag = mat2cell(J_body(:, floating_indices, :), size(J_body, 1), length(floating_indices), ones(1, nposes));
    dbody_errdfloating_states = blkdiag(dbody_errdfloating_states_diag{:});
    dfdfloating_states = dfdfloating_states + scales{i} * 2 * body_err' * dbody_errdfloating_states;
  end
  
  if nargout > 2
    marker_residuals{i} = x_body - motion_capture_data{i};
  end
end

g = [dfdq_correction_params dfdbody_params{:} dfdfloating_states];



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