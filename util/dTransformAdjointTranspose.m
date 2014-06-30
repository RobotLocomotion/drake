function ret = dTransformAdjointTranspose(H, X, dHdq, dXdq)
R = H(1:3, 1:3);
p = H(1:3, 4);

dR = getSubMatrixGradient(dHdq, 1:3, 1:3, size(H));
dp = getSubMatrixGradient(dHdq, 1:3, 4, size(H));
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