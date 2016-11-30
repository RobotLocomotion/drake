function [r_rot,dr_rot] = quatRotateVec(q,r)
% [r_rot,dr_rot] = quatRotateVec(q,r,dq,dr) Rotate vector by quaterion (with gradient)
%
% @param q - A quaternion
% @param r - A vector
% @retval r_rot - The vector given by rotating r by q
% @retval dr_rot - The Jacobian of r_rot wrt [q;r]

% these are slow and will if they are not met then an error will occur below
%sizecheck(q,[4,1]);
%sizecheck(r,[3,1]);

if nargout > 1
  dq = [eye(4),zeros(4,3)];
  dr = [zeros(4),[zeros(1,3);eye(3)]];
  [q_times_r,dq_times_r] = quatProduct(q,[0;r]);
  [q_conj, dq_conj] = quatConjugate(q);
  [r_rot,dr_rot] = quatProduct(q_times_r,q_conj);
  dr_rot = dr_rot*[dq_times_r*[dq;dr];dq_conj*dq];
  r_rot(1,:) = [];
  dr_rot(1,:) = [];
else
  r_rot = quatProduct(quatProduct(q,[0;r]),quatConjugate(q));
  r_rot(1,:) = [];
end

end
