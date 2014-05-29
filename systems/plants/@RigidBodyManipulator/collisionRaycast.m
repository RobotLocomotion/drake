function distance = collisionRaycast(obj, kinsol, origin, ray_endpoint)
% function distance = collisionRaycast(obj,kinsol, origin, point_on_ray)
%
% Uses bullet to perform a raycast, returning the distance or -1 on no hit. 
%
% See systems/plants/test/bullet_raycast_test.m for example usage.
%
% @param kinsol result of calling doKinematics(obj, q) where q is a
%   position vector.
% @param origin vector of size 3 indicating the origin of the ray
% @param ray_endpoint vector of size 3 indicating the end point of the ray,
%   specifying the direction and maximum length of the raycast.
%
% @retval distance distance to the nearest hit on the ray, or -1 on no
%    collision.

checkDependency('bullet');

if (obj.mex_model_ptr == 0)
  error('RigidBodyManipulator:collisionRaycast:MexPtrZero', ...
    'Collision raycast requires a mex model (obj.mex_model_ptr == 0 for you)');
end

if ~isstruct(kinsol)  
  % treat input as collisionDetect(obj,q)
  kinsol = doKinematics(obj,kinsol);
end

if (kinsol.mex ~= true) 
  error('RigidBodyManipulator:collisionRaycast:kinsolMex', ...
    'Call doKinematics using mex before proceeding (got kinsol.mex ~= true).');
end

distance = collisionRaycastmex(obj.mex_model_ptr, origin, ray_endpoint);