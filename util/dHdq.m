function ret = dHdq(H, S, qdotToV)
% S: motion subspace: dT/dv
% where T is the twist across the joint and v is the joint velocity vector
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
% ret = kron(H', eye(4)) * L * S * qdotToV;
M = [vectorToSkewSymmetric(-H(1:3, 1)) zeros(3);
     zeros(1, 6);
     vectorToSkewSymmetric(-H(1:3, 2)) zeros(3);
     zeros(1, 6);
     vectorToSkewSymmetric(-H(1:3, 3)) zeros(3);
     zeros(1, 6);
     vectorToSkewSymmetric(-H(1:3, 4)) eye(3);
     zeros(1, 6)];
ret = M * S * qdotToV;

end