function [Jdot_times_v, dJdot_times_v] = geometricJacobianDotV(obj, kinsol, base, endEffector, expressedIn)
% GEOMETRICJACOBIANDOTV computes the 'convective term' d/dt(J) * v, where J
% is a geometric Jacobian and v is the vector of joint velocities across
% the same joints that the geometric Jacobian spans
% @param kinsol solution structure obtained from doKinematics
% @param twists twists of all links with respect to base, expressed in body
% frame
% @param base base frame of the geometric Jacobian
% @param endEffector end effector frame of the geometric Jacobian
% @param expressedIn frame in which the geometric Jacobian is expressed
% @retval d/dt(J) * v

% Implementation makes use of the fact that
% d/dt(twist) = J * vdot + Jdot * v
% If we set vdot to zero, then Jdot * v can be found using any algorithm
% that computes the relative spatial acceleration between base and
% endEffector, expressed in expressedIn.
% The relative spatial acceleration is the sum of the spatial accelerations
% across the individual joints (with vdot set to zero), transformed to
% expressedIn frame before addition.

compute_gradient = nargout > 1;

Jdot_times_v = kinsol.JdotV{endEffector} - kinsol.JdotV{base};
if compute_gradient
  dJdot_times_v = kinsol.dJdotVdq{endEffector} - kinsol.dJdotVdq{base};
  [Jdot_times_v, dJdot_times_v] = transformSpatialAcceleration(Jdot_times_v, kinsol.T, kinsol.twists, base, endEffector, 1, expressedIn, dJdot_times_v, kinsol.dTdq, kinsol.dtwistsdq);
else
  Jdot_times_v = transformSpatialAcceleration(Jdot_times_v, kinsol.T, kinsol.twists, base, endEffector, 1, expressedIn);
end


end