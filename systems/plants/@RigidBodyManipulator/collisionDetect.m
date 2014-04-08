function [phi,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol, ...
                                          allow_multiple_contacts, ...
                                          active_collision_options)
% function [distance,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol,active_collision_options)
%
% Uses bullet to find the points of closest approach between all pairs of
% collision elements in the subset of the manipulator's collision elements
% specified by the user, with the following exceptions: 
%   * any body and it's % parent in the kinematic tree
%   * body A and body B, where body A belongs to collision filter groups that
%     ignore all of the collision filter groups to which body B belongs
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
% @ingroup Collision

if ~isstruct(kinsol)  
  % treat input as collisionDetect(obj,q)
  kinsol = doKinematics(obj,kinsol);
end

if (kinsol.mex ~= true) 
  doKinematics(obj,double(kinsol.q));
  warning('RigidBodyManipulator:collisionDetect:doKinematicsMex', ...
    'Calling doKinematics using mex before proceeding');
end

if nargin < 3, allow_multiple_contacts = false; end
if nargin < 4, active_collision_options = struct(); end
if isfield(active_collision_options,'body_idx')
  active_collision_options.body_idx = int32(active_collision_options.body_idx);
end
[xA,xB,normal,distance,idxA,idxB] = collisionDetectmex(obj.mex_model_ptr,allow_multiple_contacts,active_collision_options);
%m = numel(idxA);

%xA_in_world = zeros(size(xA));
%xB_in_world = zeros(size(xB));
%for i = 1:obj.getNumBodies()
  %xA_in_world(:,idxA==i) = forwardKin(obj,kinsol,i,xA(:,idxA==i),0);
  %xB_in_world(:,idxB==i) = forwardKin(obj,kinsol,i,xB(:,idxB==i),0);
%end

%phi = dot(normal, xA_in_world-xB_in_world);
phi = distance';

if ~isempty(obj.terrain) && ~isa(obj.terrain,'RigidBodyFlatTerrain')
  % For each point on the manipulator that can collide with non-flat terrain,
  % find the closest point on the terrain geometry
  terrain_contact_point_struct = getTerrainContactPoints(obj);

  if ~isempty(terrain_contact_point_struct)
    xA_new = [terrain_contact_point_struct.pts];
    idxA_new = cell2mat(arrayfun(@(x)repmat(x.idx,1,size(x.pts,2)), ...
                                 terrain_contact_point_struct, ...
                                 'UniformOutput',false));

    xA_new_in_world = ...
      cell2mat(arrayfun(@(x)forwardKin(obj,kinsol,x.idx,x.pts), ...
      terrain_contact_point_struct, 'UniformOutput',false));

    % Note: only implements collisions with the obj.terrain so far
    [phi_new,normal_new,xB_new,idxB_new] = ...
      collisionDetectTerrain(obj,xA_new_in_world);

    phi = [phi;phi_new];
    normal = [normal,normal_new];
    xA = [xA,xA_new];
    idxA = [idxA,idxA_new];
    xB = [xB,xB_new];
    idxB = [idxB,idxB_new];
  end
end

