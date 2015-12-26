function [com_Jacobian_dot_times_v, dcom_Jacobian_dot_times_v] = ...
  centerOfMassJacobianDotTimesV(obj,kinsol,robotnum)
% computes the quantity Jd * v, where J is the center of mass Jacobian in terms of v,
% i.e. the matrix that maps v to CoM velocity
%
% @see centroidalMOmentumMatrixDotTimesV
%
% @param kinsol solution structure obtained from doKinematics
% @param robotnum an int array. Jdot * v for the bodies that belong to
% robot(robotnum) is computed. Default is 1.
%
% @retval com_Jacobian_dot_times_v Jdot * v
% @retval dcom_Jacobian_dot_times_v gradient with respect to q

compute_gradients = nargout > 1;
if(nargin < 3)
  robotnum = 1;
end

if (kinsol.mex)
  com_Jacobian_dot_times_v = centerOfMassJacobianDotTimesVmex(obj.mex_model_ptr, kinsol.mex_ptr, robotnum - 1);
  if kinsol.has_gradients
    [com_Jacobian_dot_times_v, dcom_Jacobian_dot_times_v] = eval(com_Jacobian_dot_times_v);
    nq = length(kinsol.q);
    dcom_Jacobian_dot_times_v = dcom_Jacobian_dot_times_v(:, 1 : nq); % for backwards compatibility
  end
else
  total_mass = getMass(obj, robotnum);
  if compute_gradients
    [Adot_times_v, dAdot_times_v] = centroidalMomentumMatrixDotTimesV(obj, kinsol, robotnum);
    dcom_Jacobian_dot_times_v = dAdot_times_v(4:6, :) / total_mass;
  else
    Adot_times_v = centroidalMomentumMatrixDotTimesV(obj, kinsol, robotnum);
  end
  com_Jacobian_dot_times_v = Adot_times_v(4:6) / total_mass;
end
end
