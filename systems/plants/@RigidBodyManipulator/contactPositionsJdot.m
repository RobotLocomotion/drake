function [xA_in_world,xB_in_world,idxA,idxB,J,Jdot] = contactPositionsJdot(obj,kinsol,varargin)
% function [contact_pos,J,dJ] = contactPositionsJdot(obj,kinsol,allow_multiple_contacts,active_collision_options)
% Compute the contact positions and Jacobians, if interested in the
% inertial frame positions of these points. Most applications should see
% the contactConstraints function instead.
%
% @param obj
% @param kinsol
% @param allow_multiple_contacts Allow multiple contacts per body pair.
%      Optional, defaults to false.
% @param active_collision_options A optional struct to determine which
%    bodies and collision groups are checked. See collisionDetect.
% 
% @retval contact_pos (3 x 2m) is the set of inertial positions of the
% contacts (nearest point on a body to another body)
% @retval J (6m x n) = d/dq contact_pos
% @retval dJ (6m x n^2) = dd/dqdq contact_pos
% @retval body_idx (2m x 1) body indices

if ~isstruct(kinsol)  
  % treat input as contactPositionsJdot(obj,q)
  kinsol = doKinematics(obj,kinsol);
end

[~,~,xA,xB,idxA,idxB] = obj.collisionDetect(kinsol,varargin{:});

[xA_in_world,xB_in_world,J,Jdot] = ...
  contactPositionsFromCollisionData(obj,kinsol,xA,xB,idxA,idxB,true);

end
