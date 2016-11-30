function [phi,Jphi] = closestDistance(obj,kinsol,varargin)
% function [phi,Jphi] = closestDistance(obj,kinsol,active_collision_options)
%
% Uses bullet to find the minimum distance between all pairs of collision
% elements in the subset of the manipulator's collision elements specified by
% the user, with the following exceptions: 
%   * any body and it's % parent in the kinematic tree 
%   * body A and body B, where body A belongs to collision filter groups that
%     ignore all of the collision filter groups to which body B belongs
%
% @param kinsol  the output from doKinematics()
% @param active_collision_options - Struct that may the following fields
%   * body_idx - vector of body indices. Only these bodies will be
%       belonging to these groups will be considered for collision
%       detection.
%   Note that the filtering based on collision_filter_groups and
%   adjacency in the kinematic tree still apply.
%   @default - Consider all bodies and all groups
% @retval phi minimum distance between each pair of bodies
% @retval Jphi the kinematic jacobian of phi
% @ingroup Collision

if nargout > 1
  [phi,~,~,~,~,~,~,~,Jphi] = contactConstraints(obj,kinsol,false,varargin{:});
else
  phi = contactConstraints(obj,kinsol,false,varargin{:});
end

end

