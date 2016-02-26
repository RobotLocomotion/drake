function [x,P,J,dP,dJ] = bodyKin(obj,kinsol,body_or_frame_ind,pts)
% computes the position of pts (given in the global frame) in the body frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkId or findFrameInd)
% @retval x the position of pts (given in the global frame) in the body frame
% @retval J the Jacobian, dxdq
% @retval P the gradient, dxdpts - useful when computing forces
% @retval dJ the gradients of the Jacobian, dJdq
%
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq, P will be a
%  ((3xm)x(3xm)) matrix, with [P1;P2;...] where Pi = dxidpts, and dJ
%  will be a ((3*m)x(nq^2)) matrix, with [dJ1,dJ2,...,dJq] where
%  dJj = dJdqj

compute_P = nargout > 1;
compute_J = nargout > 2;
compute_dP = nargout > 3;
compute_dJ = nargout > 4;

kin_options.base_or_frame_id = body_or_frame_ind;
kin_options.rotation_type = 0;
if compute_dJ
  [x, J, dJ] = forwardKin(obj, kinsol, 1, pts, kin_options);
elseif compute_J
  [x, J] = forwardKin(obj, kinsol, 1, pts, kin_options);
else
  x = forwardKin(obj, kinsol, 1, pts, kin_options);
end

m = size(pts,2);
if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  P = forwardKinPositionGradientmex(obj.mex_model_ptr, kinsol.mex_ptr, m, body_or_frame_ind - 1, 0);
  if kinsol.has_gradients
    [P, dP] = eval(P);
    if isempty(dP)
      % This happens when we transform to the same frame: derivatives
      % vector never gets resized in C++ because the relative transform
      % does not depend on q or v.
      % This is the best place to fix it, because in C++ we don't have
      % the convention that AutoDiff derivative vectors always correspond
      % to q and/or v.
      dP = zeros(numel(P), length(kinsol.q));
    end
  end
else
  if compute_P
    if compute_dP
      [invT, dinvT] = relativeTransform(obj, kinsol, body_or_frame_ind, 1);
      nq = size(kinsol.q,1);
      dP = zeros((3 * m)^2, nq);
    else
      invT = relativeTransform(obj, kinsol, body_or_frame_ind, 1);
    end
    P = zeros(3*m,3*m)*kinsol.q(1); % for Taylorvar
    for i = 1 : size(pts,2)
      rows_cols = (i - 1) * 3 + 1 : i * 3;
      invR = invT(1:3, 1:3);
      P(rows_cols, rows_cols) = invR;
      if compute_dP
        dinvR = getSubMatrixGradient(dinvT, 1:3, 1:3, size(invT));
        dP = setSubMatrixGradient(dP, dinvR, rows_cols, rows_cols, size(P));
      end
    end
  end
end

end

