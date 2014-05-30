function ret = dHdq(H, S, qdotToV)
% DHDQ Computes the gradient of a homogeneous transform H with respect to
% the vector of coordinates q that determine H.
% @param H the homogeneous transform describing the configuration of `body'
% in `base' frame (i.e. H maps vectors in body to base)
% @param S the motion subspace between `body' and `base', i.e. the mapping
% from the vector of joint velocities describing the motion of `body' with
% respect to `base' to the twist of `body' with respect to `base',
% expressed in `body' frame
% @param qdotToV matrix describing the mapping from the derivative of the
% coordinate vector, qdot to the velocity vector, v.

% L = [0     0     0     0     0     0;
%      0     0     1     0     0     0;
%      0    -1     0     0     0     0;
%      0     0     0     0     0     0;
%      0     0    -1     0     0     0;
%      0     0     0     0     0     0;
%      1     0     0     0     0     0;
%      0     0     0     0     0     0;
%      0     1     0     0     0     0;
%     -1     0     0     0     0     0;
%      0     0     0     0     0     0;
%      0     0     0     0     0     0;
%      0     0     0     1     0     0;
%      0     0     0     0     1     0;
%      0     0     0     0     0     1;
%      0     0     0     0     0     0];

% S in body frame:
% ret = kron(eye(4), H) * L * S * qdotToV;
% Equivalently:
zero = zeros(4, 1);
RTilde = [H(1:3, 1:3); zeros(1, 3)];
[Rx, Ry, Rz] = deal(RTilde(:, 1), RTilde(:, 2), RTilde(:, 3));
M1 = [zero, -Rz, Ry;
      Rz, zero, -Rx;
     -Ry, Rx, zero];
M = blkdiag(M1, RTilde);
ret = M * S * qdotToV;

% S in world frame:
% ret = kron(H', eye(4)) * L * S * qdotToV;
% Equivalently:
% M = [vectorToSkewSymmetric(-H(1:3, 1)) zeros(3);
%      zeros(1, 6);
%      vectorToSkewSymmetric(-H(1:3, 2)) zeros(3);
%      zeros(1, 6);
%      vectorToSkewSymmetric(-H(1:3, 3)) zeros(3);
%      zeros(1, 6);
%      vectorToSkewSymmetric(-H(1:3, 4)) eye(3);
%      zeros(1, 6)];
% ret = M * S * qdotToV;

end