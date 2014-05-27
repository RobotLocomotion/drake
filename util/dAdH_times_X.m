function ret = dAdH_times_X(H, X, dHdq, dXdq)
R = H(1:3, 1:3);
p = H(1:3, 4);
pHat = vectorToSkewSymmetric(p);

R_selector = false(4, 4);
R_selector(1:3, 1:3) = true;
dRdq = dHdq(R_selector(:), :);

p_selector = false(4, 4);
p_selector(1:3, 4) = true;
dpdq = dHdq(p_selector(:), :);

pToPHatVec = [vectorToSkewSymmetric([-1; 0; 0]);
              vectorToSkewSymmetric([0; -1; 0]);
              vectorToSkewSymmetric([0; 0; -1])];
dpHatdq = pToPHatVec * dpdq;

Xomega_selector = false(size(X, 1), size(X, 2));
Xomega_selector(1:3, :) = true;
dXomegadq = dXdq(Xomega_selector(:), :);

Xv_selector = false(size(X, 1), size(X, 2));
Xv_selector(4:6, :) = true;
dXvdq = dXdq(Xv_selector(:), :);

Xomega = X(1:3, :);
Xv = X(4:6, :);

dRXomegadq = matGradMultMat(R, Xomega, dRdq, dXomegadq);
dRXvdq = matGradMultMat(R, Xv, dRdq, dXvdq);
dpHatRXomegadq = matGradMultMat(pHat, R * Xomega, dpHatdq, dRXomegadq);
ret = interleaveRows([3 3], {dRXomegadq, dpHatRXomegadq + dRXvdq});
end