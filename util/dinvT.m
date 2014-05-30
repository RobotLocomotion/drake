function ret = dinvT(T, dT)
% ret = dinvT(T,dT) - Invert homogeneous transform
% returns the Jacobian of the inverse of the homogeneous transform T
%
% @param T a 4x4 array representing a homogenous transform
% @param dT the Jacobian of T w.r.t. an N-element vector q in the format
% dT(:)/dq

nq = size(dT, 2);

R = T(1:3, 1:3);
p = T(1:3, 4);

R_selector = false(4, 4);
R_selector(1:3, 1:3) = true;
dR = dT(R_selector(:), :);

p_selector = false(4, 4);
p_selector(1:3, 4) = true;
dp = dT(p_selector(:), :);

dinvT_rot = transposeGrad(dR, size(R));

% dinvT_trans = -R'*dp -kron(p', eye(3)) * dinvT_rot;
% Equivalently:
dinvT_trans = -R'*dp -p(1) * dinvT_rot(1:3, :) - p(2) * dinvT_rot(4:6, :) - p(3) * dinvT_rot(7:9, :);

% ret = [interleaveRows([3 1], {dinvT_rot, zeros(3, nq)}); dinvT_trans; zeros(1, nq)];
ret = [dinvT_rot(1:3, :);
       zeros(1, nq);
       dinvT_rot(4:6, :);
       zeros(1, nq);
       dinvT_rot(7:9, :);
       zeros(1, nq);
       dinvT_trans;
       zeros(1, nq)];
end