function [distance, normal] = collisionRaycast(obj, kinsol, origins, ray_endpoints, ...
  use_margins)
% function distance = collisionRaycast(obj,kinsol, origin, point_on_ray)
%
% Uses bullet to perform a raycast, returning the distance or -1 on no hit. 
%
% See systems/plants/test/bullet_raycast_test.m for example usage.
%
% @param kinsol result of calling doKinematics(obj, q) where q is a
%   position vector.
% @param origin vector of size indicating the origin of the ray. Size is
%   3 x N.
% @param ray_endpoint vector of size 3 indicating the end point of the ray,
%   specifying the direction and maximum length of the raycast.
%   Size is 3 x N.
% @param use_margins boolean indicating whether or not to use a collision
%   model whose boxes and meshes are padded with a margin to improve
%   numerical stability of contact gradient. Default true.
%
% @retval distance distance to the nearest hit on the ray, or -1 on no
%    colliion.
% @retval normal normal vector at collision surface, or [0;0;0] if no
%    collision.

if (nargin < 5)
  use_margins = true;
end

checkDependency('bullet');

if (obj.mex_model_ptr == 0)
  error('RigidBodyManipulator:collisionRaycast:MexPtrZero', ...
    'Collision raycast requires a mex model (obj.mex_model_ptr == 0 for you)');
end

if ~isstruct(kinsol)  
  % treat input as collisionRaycast(obj,q,origin,ray_endpoint)
  kinsol = doKinematics(obj,kinsol);
end

if (kinsol.mex ~= true) 
  error('RigidBodyManipulator:collisionRaycast:kinsolMex', ...
    'Call doKinematics using mex before proceeding (got kinsol.mex ~= true).');
end

[distance, normal] = collisionRaycastmex(obj.mex_model_ptr, kinsol.mex_ptr, origins, ray_endpoints, use_margins);
