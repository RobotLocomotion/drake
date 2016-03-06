% Computes center of pressure and normal torque at computed center of
% pressure given force, torque and a contact plane defined by a normal and
% a point  on the plane. All input variables should be defined in the same
% frame, and the output will be expressed in this frame as well.
% Multiple CoPs can be computed at once by passing in multi-column inputs.
%
% @param torque torque exerted at a given contact plane
% @param force force exerted at a given contact plane
% @param normal unit normal to the contact plane. Direction (inward or
% outward) defines sign of normal torque according to the right hand rule
% @param point_on_contact_plane an arbitrary point on the contact plane.
%
% @retval cop the center of pressure. If there is no force normal to the
% surface, nan(3,n) will be returned for n-column input.
% @retval normal_torque_at_cop scalar normal component of the torque
% at the computed cop. If there is no force normal to the surface, nan is
% returned (since there is no cop at which to compute the normal torque).

function [cop, normal_torque_at_cop] = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane)
if any(abs(dot(normal, normal) - 1) > 1e-12)
  error('cols of normal should be a unit vectors')
end

n = size(torque, 2);
cop = nan(3, n);
normal_torque_at_cop = nan(1, n);

fz = dot(normal, force);
cop_exists = abs(fz) > 1e-12;
torque = torque(:, cop_exists);
force = force(:, cop_exists);
normal = normal(:, cop_exists);
point_on_contact_plane = point_on_contact_plane(:, cop_exists);
fz = fz(:, cop_exists);

torque_at_point_on_contact_plane = torque - cross(point_on_contact_plane, force);
normal_torque_at_point_on_contact_plane = dot(normal, torque_at_point_on_contact_plane);
tangential_torque = torque_at_point_on_contact_plane - bsxfun(@times, normal, normal_torque_at_point_on_contact_plane);
cop_compact = bsxfun(@rdivide, cross(normal, tangential_torque), fz) + point_on_contact_plane;
cop(:, cop_exists) = cop_compact;

if nargout > 1
  torque_at_cop = torque - cross(cop_compact, force);
  normal_torque_at_cop(:, cop_exists) = dot(normal, torque_at_cop);
end

end