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

i = [7 10 3 9 2 5 13 14 15];
j = [1 1 2 2 3 3 4 5 6];
s = [1 -1 -1 1 1 -1 1 1 1];
L = sparse(i, j, s, 16, 6);

% S in body frame:
ret = kron(speye(4), sparse(H)) * L * S * qdotToV;

% S in world frame:
% ret = kron(H', eye(4)) * L * S * qdotToV;

end