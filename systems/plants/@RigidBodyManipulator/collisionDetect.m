function [phi,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol, ...
                                          allow_multiple_contacts, ...
                                          active_collision_options)
% @param obj
% @param kinsol
% @param allow_multiple_contacts - Logical indicating whether or not the
%   collision detection algorithm may choose how many contact points to
%   return.
% @param active_collision_options - Struct that may the following fields
%   * body_idx - vector of body indices. Only these bodies will be
%       considered for collsion detection
%   * collision_groups - cell array of strings. Only the contact shapes
%       belonging to these groups will be considered for collision
%       detection.
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

