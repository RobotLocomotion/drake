function [dq, marker_params, floating_states, objective_value, marker_residuals, info] = jointOffsetCalibration(p, q_data, joint_indices,...
    bodies, marker_functions, num_marker_function_parameters, motion_capture_data, scales, options)
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

if nargin < 9
  options = struct();
end

[dq, marker_params, floating_states, objective_value, marker_residuals, info] = motionCaptureJointCalibration(...
  p, @offsetCorrectionFun, q_data, joint_indices,...
  bodies, marker_functions, num_marker_function_parameters, motion_capture_data, scales, options);

end

function [q_data_mod, dq_data_mod] = offsetCorrectionFun(q_data, q_offset)
q_data_mod = q_data + repmat(q_offset,1,size(q_data, 2));
dq_data_mod = repmat(eye(length(q_offset)), [size(q_data, 2), 1]);
end