function [phi,Jphi] = pairwiseContactDistance(obj,kinsol,body_indA,body_indB)

% Uses bullet to perform collision detection between rigid body A and rigid
% body B, and returns a single penetration constraint
%
% @param kinsol  the output from doKinematics()
% @param body_indA  numerical index of rigid body A or 
%     (less efficient:) a rigidbody object
% @param body_indB  numerical index of rigid body B (or the rigidbody object)
%
% @retval phi 1 if no contact; sum of the penetration depths if there is
% contact
% @retval Jphi the kinematic jacobian of phi
% @ingroup Collision

if (nargout>1)
  [ptsA,ptsB,~,JA,JB] = pairwiseContactTest(obj,kinsol,body_indA,body_indB);
else
  [ptsA,ptsB] = pairwiseContactTest(obj,kinsol,body_indA,body_indB);
end

if isempty(ptsA)
  phi = 1;
  Jphi = zeros(1,getNumPositions(obj));
else
  p = ptsA(:)-ptsB(:);
  phi = -p'*p;
  
  if (nargout>1)
    Jphi = -2*p'*(JA-JB);
  end
end

end