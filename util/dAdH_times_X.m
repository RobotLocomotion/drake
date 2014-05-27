function ret = dAdH_times_X(H, dHdq, X, dXdq)
nx = size(X, 2);

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

dRXomegadq = kron(eye(nx), R) * dXomegadq + kron(Xomega', eye(3)) * dRdq;
dRXvdq = kron(eye(nx), R) * dXvdq + kron(Xv', eye(3)) * dRdq;
dpHatRXomegadq = kron(eye(nx), pHat) * dRXomegadq + kron((R * Xomega)', eye(3)) * dpHatdq;

% ret = [dRXomegadq; dpHatRXomegadq + dRXvdq]; % TODO: only works for nx = 1, need to do row swap thing
ret = interleaveRows([3 3], {dRXomegadq, dpHatRXomegadq + dRXvdq});
end