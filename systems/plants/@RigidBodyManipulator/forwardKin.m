function [x, J, dJ] = forwardKin(obj, kinsol, body_or_frame_ind, points, rotation_type)
% computes the position of pts (given in the body frame) in the global frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd)
% @param rotation_type integer flag indicated whether rotations and
% derivatives should be computed (0 - no rotations, 1 - rpy, 2 - quat)
% @retval x the position of pts (given in the body frame) in the global
% frame. For rotation output, see below.
% @retval J the Jacobian, dxdq
%
% rotation_type  -- 0, no rotation included
%                -- 1, output Euler angle
%                -- 2, output quaternion
% if rotation_type = 0:
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% if rotation_type = 1:
% x will be a 6xm matrix and (following our gradient convention) J will be 
% a ((6xm)x(q)) matrix, with [J1;J2;...;Jm] where Ji = dxidq
% if rotation_type = 2:
% x will be a 7xm matrix and (following out gradient convention) J will be
% a ((7xm)*(q)) matrix with [J1;J2;....;Jm] where Ji = dxidq

compute_jacobian = nargout > 1;
compute_gradient = nargout > 2;
if (nargin<5), rotation_type=0; end

if compute_jacobian
  if compute_gradient
    [x, Jv, ~, dJv] = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotation_type, 1);
  else
    [x, Jv] = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotation_type, 1);
  end
  J = Jv * kinsol.qdotToV;
  if compute_gradient
    dJ = matGradMultMat(Jv, kinsol.qdotToV, dJv, kinsol.dqdotToVdq);
  end
else
  x = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotation_type, 1);
end

end
