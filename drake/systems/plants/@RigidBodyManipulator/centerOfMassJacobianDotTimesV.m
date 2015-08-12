function [com_Jacobian_dot_times_v, dcom_Jacobian_dot_times_v] = ...
  centerOfMassJacobianDotTimesV(obj,kinsol,robotnum)
% computes the quantity Jd * v == d(dcom/dq) * qd, where J is the
% second output of centerOfMassJacobianV.
%
% @see centroidalMOmentumMatrixDotTimesV
%
% @param kinsol solution structure obtained from doKinematics
% @param robotnum an int array. Jdot * v for the bodies that belong to
% robot(robotnum) is computed. Default is 1.
%
% @retval com_Jacobian_dot_times_v Jdot * v == d(dcom/dq) * qdot
% @retval dcom_Jacobian_dot_times_v gradient with respect to q

compute_gradients = nargout > 1;
if(nargin < 3)
  robotnum = 1;
end

if (kinsol.mex)
  if compute_gradients
    [com_Jacobian_dot_times_v, dcom_Jacobian_dot_times_v] = centerOfMassJacobianDotTimesVmex(obj.mex_model_ptr, robotnum);
  else
    [com_Jacobian_dot_times_v] = centerOfMassJacobianDotTimesVmex(obj.mex_model_ptr, robotnum);
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
