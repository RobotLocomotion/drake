function [phi,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol, ...
                                          allow_multiple_contacts, ...
                                          active_collision_options)
% @param obj
% @param kinsol
% @param allow_multiple_contacts - Logical indicating whether or not the
%   collision detection algorithm return multiple contact points for a
%   single pair of contact elements.
%   @default false
% @param active_collision_options - Struct that may the following fields
%   * body_idx - vector of body indices. Only these bodies will be
%       considered for collsion detection
%   * collision_groups - cell array of strings. Only the contact shapes
%       belonging to these groups will be considered for collision
%       detection.
%   Note that the filtering based on collision_filter_groups and
%   adjacency in the kinematic tree still apply.
%   @default - Consider all bodies and all groups
% @retval phi - (m x 1)  Vector of gap function values (typically contact
%   distance), for m possible contacts
% @retval normal - (3 x m) Contact normal vector in world coordinates,
%   points from B to A
% @retval xA - (3 x m) The closest point on body A to contact with body
%   B, relative to body A origin and in body A frame
% @retval xB - (3 x m) The closest point on body B to contact with body
%   A, relative to body B origin and in body B frame
% @retval idxA - (m x 1) The index of body A.
% @retval idxB - (m x 1) The index of body B.

if ~isstruct(kinsol)  
  % treat input as closestPointsAllBodies(obj,q)
  kinsol = doKinematics(obj,kinsol);
end

if (kinsol.mex ~= true) 
  doKinematics(obj,kinsol.q);
  warning('RigidBodyManipulator:collisionDetect:doKinematicsMex', ...
    'Calling doKinematics using mex before proceeding');
end

if nargin < 3
  allow_multiple_contacts = false;
end

assert(~allow_multiple_contacts,['Multiple contacts between a single'...
  'pair of collision elements are not yet supported.']);
[xA,xB,normal,distance,idxA,idxB] = collisionmex(obj.mex_model_ptr);
m = numel(idxA);

%xA_in_world = zeros(size(xA));
%xB_in_world = zeros(size(xB));
%for i = 1:obj.getNumBodies()
  %xA_in_world(:,idxA==i) = forwardKin(obj,kinsol,i,xA(:,idxA==i),0);
  %xB_in_world(:,idxB==i) = forwardKin(obj,kinsol,i,xB(:,idxB==i),0);
%end

%phi = dot(normal, xA_in_world-xB_in_world);
phi = distance';
