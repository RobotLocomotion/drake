function [k, body1_params, body2_params, floating_states, residuals, info, J, body1_resids, body2_resids] = jointStiffnessCalibration(p, q_data, u_data, joint_indices,...
    body1, body1_marker_fun, body1_num_params, body1_data, ...
    body2, body2_marker_fun, body2_num_params, body2_data, k_initial, options)
% Perform joint stiffness calibration, given vicon and joint data.
% Given (x,y,z) position data of some set of markers on two different
% bodies and nominal joint angles, attemps to fit three sets of parameters:
%  (1) The joint stiffnesses, such that q(t) = q(t) + k * tau(t) for all t
%  (2) Parameters for the locations of the markers on both bodies
%  (3) The floating base state of each sample in time
% 
% @param p Robot plant
% @param q_data (nxN) joint data
% @param joint_indices indices of joints for which to calibrate stiffness
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
% @param k_initial initial guess for joint stiffnesses
%
% @return dq The joint stiffnesses
% @return body1_params parameters found for body1_marker_fun
% @return body2_params parameters found for body2_marker_fun
% @return floating_states (6xN) Floating base states found for the robot
% @return residuals Residual cost
% @return info As returned by fminunc
% @return J A jacobian

if nargin < 14
  options = struct();
end

B = p.getB();
B_calibrated_joints = B(joint_indices, :);
is_simple = all(sum(B_calibrated_joints ~= 0, 1) <= 1) && all(sum(B_calibrated_joints ~= 0, 2) == 1);
if ~is_simple
  error('Function can only handle the case where B(Joint_indices, :) is simple')
end
[rows, u_indices] = find(B_calibrated_joints ~= 0);
[~, sort_indices] = sort(rows);
u_indices = u_indices(sort_indices);

tau_data = B(joint_indices, u_indices) * u_data(u_indices, :);

k_inv_sqrt_initial = 1 ./ sqrt(k_initial);
options.initial_guess = k_inv_sqrt_initial;

[k_inv_sqrt, body1_params, body2_params, floating_states, residuals, info, J, body1_resids, body2_resids] = motionCaptureJointCalibration(...
  p, @(q_data, k_inv_sqrt) stiffnessCorrectionFun(q_data, k_inv_sqrt, tau_data), q_data, joint_indices,...
  body1, body1_marker_fun, body1_num_params, body1_data, ...
  body2, body2_marker_fun, body2_num_params, body2_data, options);

k = 1 ./ (k_inv_sqrt.^2);

end

function [q_data_mod, dq_data_mod] = stiffnessCorrectionFun(q_data, k_inv_sqrt, tau_data)
k_inv = k_inv_sqrt.^2;
K_inv = diag(k_inv);
q_data_mod = q_data + K_inv * tau_data;

nk = length(k_inv);
dk_inv = 2 * diag(k_inv_sqrt);
dKInv = sparse(sub2ind(size(K_inv), 1:nk, 1:nk), 1:nk, ones(length(nk))) * dk_inv;
dq_data_mod = matGradMultMat(K_inv, tau_data, dKInv, sparse(numel(tau_data), nk));
end