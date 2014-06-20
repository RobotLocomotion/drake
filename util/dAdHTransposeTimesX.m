function ret = dAdHTransposeTimesX(H, X, dHdq, dXdq)
R = H(1:3, 1:3);
p = H(1:3, 4);

dRdq = getSubMatrixGradient(dHdq, 1:3, 1:3, size(H));
dpdq = getSubMatrixGradient(dHdq, 1:3, 4, size(H));
pHat = vectorToSkewSymmetric(p);
dpHatdq = dvectorToSkewSymmetric(dpdq);

dRTransposedq = transposeGrad(dRdq, size(R));

Xomega = X(1:3, :);
Xv = X(4:6, :);

dXomegadq = getSubMatrixGradient(dXdq, 1:3, 1:size(X,2), size(X));
dXvdq = getSubMatrixGradient(dXdq, 4:6, 1:size(X,2), size(X));

dRTransposeXomegadq = matGradMultMat(R', Xomega, dRTransposedq, dXomegadq);
dRTransposeXvdq = matGradMultMat(R', Xv, dRTransposedq, dXvdq);
dRTransposepHatdq = matGradMultMat(R', pHat, dRTransposedq, dpHatdq);
dRTransposepHatXvdq = matGradMultMat(R' * pHat, Xv, dRTransposepHatdq, dXvdq);
ret = interleaveRows([3 3], {dRTransposeXomegadq - dRTransposepHatXvdq, dRTransposeXvdq});
end