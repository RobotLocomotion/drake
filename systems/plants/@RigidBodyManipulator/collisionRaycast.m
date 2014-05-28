function distance = collisionRaycast(obj, kinsol, origin, ray_endpoint)
% function distance = collisionRaycast(obj,kinsol, origin, point_on_ray)
%
% Uses bullet to perform a raycast, returning the distance or -1 on no hit. 
%
% @param kinsol result of calling doKinematics(obj, q) where q is a
%   position vector.  Can also be q, and we'll call doKinematics for you.
% @param origin vector of size 3 indicating the origin of the ray
% @param ray_endpoint vector of size 3 indicating the end point of the ray,
%   specifying the direction and maximum length of the raycast.
%
% @retval distance distance to the nearest hit on the ray, or -1 on no
%    collision.

if ~isstruct(kinsol)  
  % treat input as collisionDetect(obj,q)
  kinsol = doKinematics(obj,kinsol);
end

if (kinsol.mex ~= true) 
  doKinematics(obj,double(kinsol.q));
  warning('RigidBodyManipulator:collisionRaycast:doKinematicsMex', ...
    'Calling doKinematics using mex before proceeding');
end

distance = collisionRaycastmex(obj.mex_model_ptr, origin, ray_endpoint);