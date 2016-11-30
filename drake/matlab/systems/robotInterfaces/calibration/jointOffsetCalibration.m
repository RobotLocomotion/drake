function [dq, marker_params, floating_states, objective_value, marker_residuals, info] = jointOffsetCalibration(p, q_data, joint_indices,...
    bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, options)
% Perform joint offset calibration, given motion capture data and joint data.
% Given (x,y,z) position data of some set of markers on various
% bodies and nominal joint angles, attemps to fit three sets of parameters:
%  (1) The joint offsets, such that q(t) = q(t) + dq for all t
%  (2) Parameters for the locations of the markers on each of the bodies
%  (3) The floating base state of each sample in time
% 
% @param p Robot plant
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
%
% @retval dq joint offsets
% @retval marker_params nb x 1 cell array where marker_params{i} contains
% the optimized parameters found for marker_functions{i}
% @retval floating_states (6 x N) floating base states found for the robot
% @retval objective_value objective value at solution
% @retval marker_residuals nb x 1 cell array where marker_residuals{i}
% contains the measurement residuals of the markers attached to bodies{i}
% @retval info info as returned by fminunc

if nargin < 9
  options = struct();
end

[dq, marker_params, floating_states, objective_value, marker_residuals, info] = motionCaptureJointCalibration(...
  p, @offsetCorrectionFun, q_data, joint_indices,...
  bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, options);

end

function [q_data_mod, dq_data_mod] = offsetCorrectionFun(q_data, q_offset)
q_data_mod = q_data + repmat(q_offset,1,size(q_data, 2));
dq_data_mod = repmat(eye(length(q_offset)), [size(q_data, 2), 1]);
end