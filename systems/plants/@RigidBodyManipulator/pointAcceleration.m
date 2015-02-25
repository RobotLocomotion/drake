function [point_accel, dpoint_accel] = pointAcceleration(obj, kinsol, body_index, point)
% computes the acceleration of a point fixed in body frame with respect to
% the world, assuming zero joint accelerations (i.e. accelerations are due
% to coriolis/centripetal effects only)

compute_gradient = nargout > 1;

twist = kinsol.twists{body_index};
omega = twist(1:3);
v = twist(4:6);

spatial_accel = kinsol.JdotV{body_index};
omegad = spatial_accel(1:3);
vd = spatial_accel(4:6);

T = kinsol.T{body_index};
R = T(1:3,1:3);
p = T(1:3,4);
point_base = R * point + p;

point_vel = cross(omega, point_base) + v;
point_accel = cross(omegad, point_base) + vd + cross(omega, point_vel);

if compute_gradient
  dtwist = kinsol.dtwistsdq{body_index};
  domega = dtwist(1:3, :);
  dv = dtwist(4:6, :);
  
  dspatial_accel = kinsol.dJdotVdq{body_index};
  domegad = dspatial_accel(1:3, :);
  dvd = dspatial_accel(4:6, :);
  
  dT = kinsol.dTdq{body_index};
  dR = getSubMatrixGradient(dT, 1:3, 1:3, size(T));
  dp = getSubMatrixGradient(dT, 1:3, 4, size(T));
  dpoint_base = matGradMult(dR, point) + dp;
  
  dpoint_vel = dcross(omega, point_base, domega, dpoint_base) + dv;
  dpoint_accel = dcross(omegad, point_base, domegad, dpoint_base) + ...
    dvd + dcross(omega, point_vel, domega, dpoint_vel);
end
end
