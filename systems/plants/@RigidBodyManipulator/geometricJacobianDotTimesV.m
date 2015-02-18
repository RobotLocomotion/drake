function [Jdot_times_v, dJdot_times_v] = geometricJacobianDotTimesV(obj, kinsol, base, end_effector, expressed_in)
% geometricJacobianDotTimesV computes the `convective term' d/dt(J) * v, 
% where J is a geometric Jacobian and v is the vector of joint velocities 
% across the joints on the path from base to end_effector.
%
% @param kinsol solution structure obtained from doKinematics
% @param base base frame of the geometric Jacobian
% @param end_effector end effector frame of the geometric Jacobian
% @param expressed_in frame in which the geometric Jacobian is expressed
%
% @retval d/dt(J) * v

compute_gradient = nargout > 1;

Jdot_times_v = kinsol.JdotV{end_effector} - kinsol.JdotV{base};
if compute_gradient
  dJdot_times_v = kinsol.dJdotVdq{end_effector} - kinsol.dJdotVdq{base};
  [Jdot_times_v, dJdot_times_v] = transformSpatialAcceleration(kinsol, base, end_effector, 1, expressed_in, Jdot_times_v, dJdot_times_v);
else
  Jdot_times_v = transformSpatialAcceleration(kinsol, base, end_effector, 1, expressed_in, Jdot_times_v);
end

end
