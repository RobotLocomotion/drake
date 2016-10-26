function [phi,Jphi,distance] = pairwiseClosestDistance(obj,kinsol,bodyA_idx,bodyB_idx)

% Uses bullet to find the minimum distance between bodies A and B
%
% @param kinsol  the output from doKinematics()
% @param bodyA_idx  numerical index of rigid body A or 
%     (less efficient:) a rigidbody object
% @param bodyB_idx  numerical index of rigid body B (or the rigidbody object)
%
% @retval phi minimum distance between bodies A and B
% @retval Jphi the kinematic jacobian of phi
% @ingroup Collision

if (nargout>1)
  [ptA,ptB,normal,distance,JA,JB] = pairwiseClosestPoints(obj,kinsol,bodyA_idx,bodyB_idx);
else
  [ptA,ptB,normal,distance] = pairwiseClosestPoints(obj,kinsol,bodyA_idx,bodyB_idx);
end

if isempty(ptA)
  phi = 1;
  Jphi = zeros(1,getNumPositions(obj));
else
  p = sign(distance)*(ptA(:)-ptB(:));
  %phi = p'*p;
  phi = distance;
  
  if (nargout>1)
    Jphi = 2*p'*(JA-JB);
  end
end

end
