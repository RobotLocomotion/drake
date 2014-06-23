function ret = dAdHTimesX(H, X, dH, dX)
R = H(1:3, 1:3);
p = H(1:3, 4);

dR = getSubMatrixGradient(dH, 1:3, 1:3, size(H));
dp = getSubMatrixGradient(dH, 1:3, 4, size(H));
[p_hat, dp_hat] = vectorToSkewSymmetric(p, dp);

Xomega = X(1:3, :);
Xv = X(4:6, :);

dXomega = getSubMatrixGradient(dX, 1:3, 1:size(X,2), size(X));
dXv = getSubMatrixGradient(dX, 4:6, 1:size(X,2), size(X));

dRXomega = matGradMultMat(R, Xomega, dR, dXomega);
dRXv = matGradMultMat(R, Xv, dR, dXv);
dpHatRXomega = matGradMultMat(p_hat, R * Xomega, dp_hat, dRXomega);
ret = interleaveRows([3 3], {dRXomega, dpHatRXomega + dRXv});
end