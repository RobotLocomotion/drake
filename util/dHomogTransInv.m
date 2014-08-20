function ret = dHomogTransInv(T, dT)
% ret = dHomogTransInv(T,dT)
% returns the Jacobian of the inverse of the homogeneous transform T
%
% @param T a 4x4 array representing a homogenous transform
% @param dT the Jacobian of T w.r.t. an N-element vector q in the format
% dT(:)/dq

nq = size(dT, 2);

R = T(1:3, 1:3);
p = T(1:3, 4);

dR = getSubMatrixGradient(dT, 1:3, 1:3, size(T));
dp = getSubMatrixGradient(dT, 1:3, 4, size(T));

dinvT_R = transposeGrad(dR, size(R));
dinvT_p = -R'*dp - matGradMult(dinvT_R, p, false);

ret = zeros(numel(T), nq);
ret = setSubMatrixGradient(ret, dinvT_R, 1:3, 1:3, size(T));
ret = setSubMatrixGradient(ret, dinvT_p, 1:3, 4, size(T));
end