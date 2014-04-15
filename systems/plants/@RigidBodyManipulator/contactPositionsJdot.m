function [contact_pos,J,Jdot] = contactPositionsJdot(obj,kinsol,varargin)
% function [contact_pos,J,dJ] = contactPositionsJdot(obj,kinsol,body_idx,body_contacts)
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
  % treat input as contactPositions(obj,q)
  kinsol = doKinematics(obj,kinsol);
end

[~,~,xA,xB,idxA,idxB] = obj.collisionDetect(kinsol,varargin{:});

nq = obj.getNumPositions;
nC = length(idxA);
body_idx = [idxA;idxB];
x_body = [xA xB];
contact_pos = zeros(3,2*nC);

if nargout > 1,
  J = zeros(6*nC,nq);
end
if nargout > 2,
  Jdot = zeros(6*nC,nq);
end

for i=1:2*nC,
  if (nargout>2)
    [contact_pos(:,i),J((1:3) + 3*(i-1),:)] = obj.forwardKin(kinsol,body_idx(i),x_body(:,i));
    Jdot((1:3) + 3*(i-1),:) = forwardJacDot(obj,kinsol,body_idx(i),x_body(:,i));
  elseif (nargout>1)
    [contact_pos(:,i),J((1:3) + 3*(i-1),:)] = obj.forwardKin(kinsol,body_idx(i),x_body(:,i));
  else
    contact_pos(:,i) = obj.forwardKin(kinsol,body_idx(i),x_body(:,i));
  end
end
end