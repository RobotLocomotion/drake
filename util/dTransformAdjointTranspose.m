function ret = dTransformAdjointTranspose(T, X, dTdq, dXdq)
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

dR = getSubMatrixGradient(dTdq, 1:3, 1:3, size(T));
dp = getSubMatrixGradient(dTdq, 1:3, 4, size(T));
[pHat, dpHatdq] = vectorToSkewSymmetric(p, dp);

dRTranspose = transposeGrad(dR, size(R));

Xomega = X(1:3, :);
Xv = X(4:6, :);

dXOmega = getSubMatrixGradient(dXdq, 1:3, 1:size(X,2), size(X));
dXv = getSubMatrixGradient(dXdq, 4:6, 1:size(X,2), size(X));

dRTransposeXomega = matGradMultMat(R', Xomega, dRTranspose, dXOmega);
dRTransposeXv = matGradMultMat(R', Xv, dRTranspose, dXv);
dRTransposepHat = matGradMultMat(R', pHat, dRTranspose, dpHatdq);
dRTransposepHatXv = matGradMultMat(R' * pHat, Xv, dRTransposepHat, dXv);
ret = interleaveRows([3 3], {dRTransposeXomega - dRTransposepHatXv, dRTransposeXv});
end