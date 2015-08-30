function [phi,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol, ...
                                          allow_multiple_contacts, ...
                                          active_collision_options)
% [distance,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol)
% returns the points of closest approach between all pairs of
% collision elements in the manipulator, with the following exceptions: 
%   * any body and it's % parent in the kinematic tree
%   * body A and body B, where body A belongs to collision filter groups that
%     ignore all of the collision filter groups to which body B belongs
% as well as the points of closest approach between the manipulator's
% terrain contact points and terrain (if applicable). For a description
% of terrain contact points, see <a href="matlab:help RigidBodyGeometry/getTerrainContactPoints">RigidBodyGeometry/getTerrainContactPoints</a>
%
% [...] = collisionDetect(obj,kinsol,active_collision_options) returns
% the same information as above, but only for those contact pairs that
% satisfy the criteria in active_collision_options (See below).
%
% @param obj
% @param kinsol result of calling doKinematics(obj, q) where q is a
%   position vector.  Can also be q, and we'll call doKinematics for you.
% @param allow_multiple_contacts - Logical indicating whether or not the
%   collision detection algorithm return multiple contact points for a
%   single pair of contact elements.
%   @default false
% @param active_collision_options - Struct that may the following fields
%   * body_idx - vector of body indices. Only these bodies will be
%       considered for collsion detection
%       @default - Consider all bodies
%   * collision_groups - cell array of strings. Only the collision geometry
%       belonging to these groups will be considered for collision
%       detection.Note that the filtering based on
%       collision_filter_groups and adjacency in the kinematic tree
%       still apply.
%       @default - Consider all groups
%   * terrain_only - Logical scalar. If true, only consider the
%       interaction between the manipulator's terrain contact points and
%       terrain. 
%       @default - false
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

if nargin < 3, allow_multiple_contacts = false; end
if nargin < 4 || isempty(active_collision_options), active_collision_options = struct(); end
if isfield(active_collision_options,'body_idx')
  active_collision_options.body_idx = int32(active_collision_options.body_idx);
end
if ~isfield(active_collision_options,'terrain_only')
  active_collision_options.terrain_only = false;
end

force_collisionDetectTerrain = ~obj.contact_options.use_bullet;


if (obj.contact_options.use_bullet && ~active_collision_options.terrain_only && obj.mex_model_ptr ~= 0 && kinsol.mex)
  [xA,xB,normal,distance,idxA,idxB] = collisionDetectmex(obj.mex_model_ptr, kinsol.mex_ptr, allow_multiple_contacts,active_collision_options);
  if isempty(idxA)
    idxA = [];
    idxB = [];
    xA = [];
    xB = [];
    distance = [];
  else
    idxA = double(idxA);
    idxB = double(idxB);
  end
  phi = distance';
else
  phi = [];
  normal = [];
  xA = [];
  idxA = [];
  xB = [];
  idxB = [];
  
  if isempty([obj.body.collision_geometry])
    % then I don't have any contact geometry.  all done.
    return;
  end
  
  if active_collision_options.terrain_only || ...
      ~isfield(active_collision_options,'collision_groups') || ...
      ismember('terrain',active_collision_options.collision_groups)
    force_collisionDetectTerrain = true;
  end
  
  if obj.mex_model_ptr == 0
    warnOnce(obj.warning_manager,'Drake:RigidBodyManipulator:collisionDetect:noMexPtr', ...
      ['This model has no mex pointer. Only checking collisions between ' ...
      'terrain contact points and terrain']);
  elseif ~kinsol.mex
    if isa(kinsol.q,'TaylorVar')
      error('Drake:RigidBodyManipulator:collisionDetect:unsupportedTaylorVar','The collision detection code runs through bullet, so TaylorVars do not work here');
    end
    warnOnce(obj.warning_manager,'Drake:RigidBodyManipulator:collisionDetect:doKinematicsMex', ...
      ['kinsol was generated with use_mex = false. Only checking collisions ' ...
      'between terrain contact points and terrain']);
  end
end

if ~isempty(obj.terrain) && ...
    (force_collisionDetectTerrain || ~isa(obj.terrain,'RigidBodyFlatTerrain'))
  % For each point on the manipulator that can collide with terrain,
  % find the closest point on the terrain geometry
  if isfield(active_collision_options,'body_idx')
    if isfield(active_collision_options,'collision_groups')
      terrain_contact_point_struct = getTerrainContactPoints(obj, ...
        active_collision_options.body_idx,...
        active_collision_options.collision_groups);
    else
      terrain_contact_point_struct = getTerrainContactPoints(obj, ...
        active_collision_options.body_idx);
    end
  else
    if isfield(active_collision_options,'collision_groups')
      terrain_contact_point_struct = getTerrainContactPoints(obj, ...
        2:obj.getNumBodies(),...
        active_collision_options.collision_groups);
    else
      terrain_contact_point_struct = getTerrainContactPoints(obj);
    end
  end

  if ~isempty(terrain_contact_point_struct)
    
    xA_new = [terrain_contact_point_struct.pts];
    numStructs = size(terrain_contact_point_struct,2);

    %total_pts = size(horzcat(terrain_contact_point_struct.pts),2); %too slow
    xA_new_in_world =  [];
    k = 1;
    for i = 1:numStructs
        numPts = size(terrain_contact_point_struct(i).pts,2);
        xA_new_in_world = [xA_new_in_world, forwardKin(obj, kinsol, ...
        terrain_contact_point_struct(i).idx, terrain_contact_point_struct(i).pts)];
        for j = 1:numPts
            terrain_idxs(k) = terrain_contact_point_struct(i).idx;
            k = k+1;
        end
    end

    % Note: only implements collisions with the obj.terrain so far
 [phi_new,normal_new,xB_new,idxB_new] = ...
      collisionDetectTerrain(obj,xA_new_in_world);

    phi = [phi;phi_new];
    normal = [normal,normal_new];
    xA = [xA,xA_new];
    idxA = [idxA,terrain_idxs];
    xB = [xB,xB_new];
    idxB = [idxB,idxB_new];
  end
end

