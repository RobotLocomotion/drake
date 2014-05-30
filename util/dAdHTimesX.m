function ret = dAdHTimesX(H, X, dHdq, dXdq)
R = H(1:3, 1:3);
p = H(1:3, 4);
pHat = vectorToSkewSymmetric(p);

dRdq = getSubMatrixGradient(dHdq, 1:3, 1:3, size(H));
dpdq = getSubMatrixGradient(dHdq, 1:3, 4, size(H));
pToPHatVec = [vectorToSkewSymmetric([-1; 0; 0]);
              vectorToSkewSymmetric([0; -1; 0]);
              vectorToSkewSymmetric([0; 0; -1])];
dpHatdq = pToPHatVec * dpdq;

Xomega = X(1:3, :);
Xv = X(4:6, :);

dXomegadq = getSubMatrixGradient(dXdq, 1:3, 1:size(X,2), size(X));
dXvdq = getSubMatrixGradient(dXdq, 4:6, 1:size(X,2), size(X));

dRXomegadq = matGradMultMat(R, Xomega, dRdq, dXomegadq);
dRXvdq = matGradMultMat(R, Xv, dRdq, dXvdq);
dpHatRXomegadq = matGradMultMat(pHat, R * Xomega, dpHatdq, dRXomegadq);
ret = interleaveRows([3 3], {dRXomegadq, dpHatRXomegadq + dRXvdq});
end