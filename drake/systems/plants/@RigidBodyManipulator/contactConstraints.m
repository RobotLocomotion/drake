function [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = contactConstraints(obj,kinsol,allow_multiple_contacts,active_collision_options)
% function [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = contactConstraints(obj,kinsol,body_idx)
% Compute the contact constraints for a manipulator, and relevent bases.
% The contact frame always points from body B to body A.
%
% @param obj
% @param kinsol
% @param allow_multiple_contacts Allow multiple contacts per body pair.
%      Optional, defaults to false.
% @param active_collision_options A optional struct to determine which
%    bodies and collision groups are checked. See collisionDetect.
% @retval phi (m x 1) Vector of gap function values (typically contact distance), for m possible contacts
% @retval normal (3 x m) Contact normal vector in world coordinates, points from B to A
% @retval d {k} (3 x m) Contact friction basis vectors in world coordinates, points from B to A
% @retval xA (3 x m) The closest point on body A to contact with body B, relative to body A origin and in body A frame
% @retval xB (3 x m) The closest point on body B to contact with body A, relative to body B origin and in body B frame
% @retval idxA (m x 1) The index of body A. 0 is the special case for the environment/terrain
% @retval idxB (m x 1) The index of body B. 0 is the special case for the environment/terrain
% @retval mu (m x 1) Coefficients of friction
% @retval n (m x n) normal vector in joint coordinates, state vector length n
% @retval D {2k}(m x n) friction cone basis in joint coordinates, for k directions
% @retval dn (mn x n) dn/dq derivative
% @retval dD {2k}(mn x n) dD/dq derivative

compute_first_derivative = nargout > 8;
compute_kinematics_gradients = nargout > 10;

if nargin<3,
  allow_multiple_contacts = false;
end

if nargin<4,
  active_collision_options = struct();
end

if ~isstruct(kinsol)
  % treat input as contactPositions(obj,q)
  kin_options = struct('compute_gradients', compute_kinematics_gradients);
  kinsol = doKinematics(obj, kinsol, [], kin_options);
end

[phi,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol,allow_multiple_contacts,active_collision_options);
idxA = idxA';
idxB = idxB';
nC = numel(phi);

% If there are no potential collisions, return empty
if nC == 0
  d = [];
  mu = [];
  n = [];
  D = [];
  dn = [];
  dD = [];
  return;
end

% For now, all coefficients of friction are 1
mu = ones(nC,1);

d = obj.surfaceTangents(normal);

if obj.mex_model_ptr ~= 0
  if compute_first_derivative
    if ~compute_kinematics_gradients
      [n, D] = contactConstraintsmex(obj.mex_model_ptr, kinsol.mex_ptr, normal, int32(idxA), int32(idxB), xA, xB, d);
    else
      [n, D, dn, dD] = contactConstraintsmex(obj.mex_model_ptr, kinsol.mex_ptr, normal, int32(idxA), int32(idxB), xA, xB, d);
    end
  end

else %MATLAB implementation
  if compute_first_derivative
    if ~compute_kinematics_gradients
      [n, D] = contactConstraintDerivatives(obj, normal, kinsol, idxA, idxB, xA, xB, d);
    else
      [n, D, dn, dD] = contactConstraintDerivatives(obj, normal, kinsol, idxA, idxB, xA, xB, d);
    end
end

end

