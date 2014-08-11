function ret = dTransformAdjointTranspose(T, X, dT, dX)
% Computes the gradient of transformAdjoint(T)' * X
%
% @param T a homogeneous transform. No checks are performed to ascertain
% that T is indeed a homogeneous transform
% @param X a 6xn matrix
% @param dT the gradient of T with respect to a vector x. dT is a
% 36 x m matrix, where m is the dimension of x
% @param dX the gradient of X with respect to a vector x. dX is a (6*n) x m
% matrix, where m is the dimension of x
%
% @retval ret the gradient of transformAdjoint(T)' * X with respect to x

R = T(1:3, 1:3);
p = T(1:3, 4);

dR = getSubMatrixGradient(dT, 1:3, 1:3, size(T));
dp = getSubMatrixGradient(dT, 1:3, 4, size(T));
[pHat, dpHat] = vectorToSkewSymmetric(p, dp);

Xomega = X(1:3, :);
Xv = X(4:6, :);

dXOmega = getSubMatrixGradient(dX, 1:3, 1:size(X,2), size(X));
dXv = getSubMatrixGradient(dX, 4:6, 1:size(X,2), size(X));

dRTranspose = transposeGrad(dR, size(R));
dRTransposeXomega = matGradMultMat(R', Xomega, dRTranspose, dXOmega);
dRTransposeXv = matGradMultMat(R', Xv, dRTranspose, dXv);
dRTransposepHat = matGradMultMat(R', pHat, dRTranspose, dpHat);
dRTransposepHatXv = matGradMultMat(R' * pHat, Xv, dRTransposepHat, dXv);
ret = interleaveRows([3 3], {dRTransposeXomega - dRTransposepHatXv, dRTransposeXv});
end