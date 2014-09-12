function [dq, body1_params, body2_params, floating_states, residuals, info, J, body1_resids, body2_resids] = jointOffsetCalibration(p, q_data, joint_indices,...
    body1, body1_marker_fun, body1_num_params, body1_data, ...
    body2, body2_marker_fun, body2_num_params, body2_data, options)
% NOTEST
% Perform joint offset calibration, given vicon and joint data.
% Given (x,y,z) position data of some set of markers on two different
% bodies and nominal joint angles, attemps to fit three sets of parameters:
%  (1) The joint offsets, such that q(t) = q(t) + dq for all t
%  (2) Parameters for the locations of the markers on both bodies
%  (3) The floating base state of each sample in time
% 
% @param p Robot plant
% @param q_data (nxN) joint data
% @param joint_indices indices of joints for which to calibrate offset
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
% @return dq The joint offsets
% @return body1_params parameters found for body1_marker_fun
% @return body2_params parameters found for body2_marker_fun
% @return floating_states (6xN) Floating base states found for the robot
% @return residuals Residual cost
% @return info As returned by fminunc
% @return J A jacobian

if nargin < 12
  options = struct();
end
if ~isfield(options,'search_floating')
  options.search_floating = true;
end

n = p.getNumStates/2;

% zero out all other joints
% q_data(setdiff(1:n,joint_indices),:) = 0*q_data(setdiff(1:n,joint_indices),:);

n_joints = length(joint_indices);
N = size(q_data,2);
m1 = size(body1_data,2);
m2 = size(body2_data,2);

% f = @(params) point_resids(p, body1, body2, q_data, joint_indices, N, n, m1, m2, n_joints, 10, 1, body1_num_params, body2_num_params, body1_marker_fun, body2_marker_fun, params(1:n_joints), params(n_joints+1:n_joints+body1_num_params), params(n_joints+body1_num_params+1:n_joints+body1_num_params+body2_num_params), reshape(params(n_joints+body1_num_params+body2_num_params+1:end),6,[]), body1_data, body2_data);
% fg = @(x) geval(f,x,struct('grad_method','numerical'));

if options.search_floating
  f = @(params) point_resids_with_grad(p, body1, body2, q_data, joint_indices, N, n, m1, m2, n_joints, 1e2, 1, body1_num_params, body2_num_params, body1_marker_fun, body2_marker_fun, params(1:n_joints), params(n_joints+1:n_joints+body1_num_params), params(n_joints+body1_num_params+1:n_joints+body1_num_params+body2_num_params), reshape(params(n_joints+body1_num_params+body2_num_params+1:end),6,[]), body1_data, body2_data);
  x0 = zeros(n_joints+6*N+body1_num_params + body2_num_params,1);
else
  f = @(params) point_resids_with_grad(p, body1, body2, q_data, joint_indices, N, n, m1, m2, n_joints, 1e2, 1, body1_num_params, body2_num_params, body1_marker_fun, body2_marker_fun, params(1:n_joints), params(n_joints+1:n_joints+body1_num_params), params(n_joints+body1_num_params+1:n_joints+body1_num_params+body2_num_params), [], body1_data, body2_data);
  x0 = zeros(n_joints+body1_num_params + body2_num_params,1);
end
fmin_options = optimset('Display','iter-detailed','GradObj','on','DerivativeCheck','on','FinDiffType','central');

% p0 = randn(n_joints+6*N+body1_num_params + body2_num_params,1);
% [x,dx] = f2(p0);
% [x2,dx2] = fg(p0);
% keyboard
[X,FVAL,EXITFLAG] = fminunc(f,x0,fmin_options);

dq = X(1:n_joints);
body1_params = X(n_joints+1:n_joints+body1_num_params);
body2_params = X(n_joints+body1_num_params+1:n_joints+body1_num_params+body2_num_params);
floating_states = reshape(X(n_joints+body1_num_params+body2_num_params+1:end),6,[]);
residuals = FVAL;
info = EXITFLAG;

[~,~,J,body1_resids,body2_resids] = f(X);
end

function [f, g, J, body1_resids, body2_resids]=point_resids_with_grad(p, body1, body2, q, joint_indices, N, n, m1, m2, n_joints, ...
    body1_scale, body2_scale, body1_num_params, body2_num_params, body1_marker_fun, body2_marker_fun, ...
    q_offset, body1_params, body2_params, floating_states, body1_data, body2_data)
q(joint_indices,:) = q(joint_indices,:) + repmat(q_offset,1,N);

if ~isempty(floating_states)
  q(1:6,:) = floating_states;
end

x_body1 = zeros(3, m1, N);
x_body2 = zeros(3 ,m2, N);

J_body1 = zeros(3 * m1, n, N);
J_body2 = zeros(3 * m2, n, N);
R_body1 = cell(N, 1);
R_body2 = cell(N, 1);

[pts_body1, dpts_body1] = body1_marker_fun(body1_params);
[pts_body2, dpts_body2] = body2_marker_fun(body2_params);

for i=1:N,
  kinsol=p.doKinematics(q(:,i));
%   x_rows = 3 * (i - 1) + 1 : 3 * i;
%   J_rows = 3 * m1 * (i - 1) + 1: 3 * m1 * i;
%   [x_body1(x_rows, :), J_body1(J_rows, :)] = p.forwardKin(kinsol,body1,pts_body1);
%   [x_body2(x_rows,:), J_body2(J_rows, :)] = p.forwardKin(kinsol,body2,pts_body2);
  [x_body1(:, :, i), J_body1(:, :, i)] = p.forwardKin(kinsol,body1,pts_body1);
  [x_body2(:, :, i), J_body2(:, :, i)] = p.forwardKin(kinsol,body2,pts_body2);  

%   % awkward hack to get out the rotation matrix
  R_body1{i} = p.forwardKin(kinsol,body1,eye(3)) - p.forwardKin(kinsol,body1,zeros(3));
  R_body2{i} = p.forwardKin(kinsol,body2,eye(3)) - p.forwardKin(kinsol,body2,zeros(3));
end

% remove obscured markers from calculation (nan)
% this will also zero out any gradient effects, since cost is purely quadratic
% df/dy * (x_body - body_data);
body1_data(isnan(body1_data)) = x_body1(isnan(body1_data));
body2_data(isnan(body2_data)) = x_body2(isnan(body2_data));

body1_err = x_body1(:) - body1_data(:);
body2_err = x_body2(:) - body2_data(:);

f = body1_scale * (body1_err' * body1_err) + body2_scale * (body2_err' * body2_err);

% Gradient computation
% dfq_offset
dbody1_errdqoffset = reshape(permute(J_body1(:, joint_indices, :), [1 3 2]), [], length(joint_indices)); % Jacobians for each pose stacked on top of each other
dbody2_errdqoffset = reshape(permute(J_body2(:, joint_indices, :), [1 3 2]), [], length(joint_indices));

dfdqoffset = body1_scale * 2 * body1_err' * dbody1_errdqoffset + body2_scale * 2 * body2_err' * dbody2_errdqoffset;

% dfdbody1_params
dbody1_err_dbody1_params = cellMatGradMultMat(R_body1, pts_body1, dpts_body1);
dfdbody1_params = body1_scale * 2 * body1_err' * dbody1_err_dbody1_params;

% dfdbody2_params
dbody2_err_dbody2_params = cellMatGradMultMat(R_body2, pts_body2, dpts_body2);
dfdbody2_params = body2_scale * 2 * body2_err' * dbody2_err_dbody2_params;

% dfdfloating_states
floating_indices = 1 : 6; % TODO: generalize

dbody1_errdfloating_states_diag = mat2cell(J_body1(:, floating_indices, :), size(J_body1, 1), length(floating_indices), ones(1, N));
dbody1_errdfloating_states = blkdiag(dbody1_errdfloating_states_diag{:});

dbody2_errdfloating_states_diag = mat2cell(J_body2(:, floating_indices, :), size(J_body2, 1), length(floating_indices), ones(1, N));
dbody2_errdfloating_states = blkdiag(dbody2_errdfloating_states_diag{:});

dbody1_err_magdfloating_states = 2 * body1_err' * dbody1_errdfloating_states;
dbody2_err_magdfloating_states = 2 * body2_err' * dbody2_errdfloating_states;

dfdfloating_states = body1_scale * dbody1_err_magdfloating_states + body2_scale * dbody2_err_magdfloating_states;

if ~isempty(floating_states)
  g = [dfdqoffset dfdbody1_params dfdbody2_params dfdfloating_states];
else
  g = [dfdqoffset dfdbody1_params dfdbody2_params];
end

% Evaluation outputs
if nargout > 2
  J_qoffset = zeros(n_joints,(m1+m2)*N*3);
  J_body1_params = zeros(body1_num_params,(m1+m2)*N*3);
  J_body2_params = zeros(body2_num_params,(m1+m2)*N*3);
  J_floating_states = zeros(6,N,(m1+m2)*N*3);
  J = zeros(body1_num_params+body2_num_params+n_joints+6*N,(m1+m2)*N*3);
  for i=1:N,
    for j=1:m1,
      J_qoffset(:,(i-1)*m1*3 + (j-1)*3 + (1:3)) = J_body1((1:3) + (j-1)*3,joint_indices,i)';
      J_body1_params(:,(i-1)*m1*3 + (j-1)*3 + (1:3)) = dpts_body1((1:3) + (j-1)*3,:)'*R_body1{i}';
      J_floating_states(:,i,(i-1)*m1*3 + (j-1)*3 + (1:3)) = J_body1((1:3) + (j-1)*3,1:6,i)';
    end
    
    for j=1:m2,
      J_qoffset(:,N*m1*3 + (i-1)*m2*3 + (j-1)*3 + (1:3)) = J_body2((1:3) + (j-1)*3,joint_indices,i)';
      J_body2_params(:,N*m1*3 + (i-1)*m2*3 + (j-1)*3 + (1:3)) = dpts_body2((1:3) + (j-1)*3,:)'*R_body2{i}';
      J_floating_states(:,i,N*m1*3 + (i-1)*m2*3 + (j-1)*3 + (1:3)) = J_body2((1:3) + (j-1)*3,1:6,i)';
    end
  end
  if ~isempty(floating_states)
    J = [J_qoffset; J_body1_params; J_body2_params; reshape(J_floating_states,[],(m1+m2)*N*3)];
  else
    J = [J_qoffset; J_body1_params; J_body2_params];
  end
end

if nargout > 3
  body1_resids = x_body1 - body1_data;
  body2_resids = x_body2 - body2_data;
end
end