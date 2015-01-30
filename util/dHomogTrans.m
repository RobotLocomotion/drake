function ret = dHomogTrans(T, S, qdot_to_v)
% Computes the gradient of a homogeneous transform T with respect to
% the vector of coordinates q that determine T.
%
% @param T the homogeneous transform describing the configuration of `body'
% with respect to `base' frame (i.e. T maps homogeneous vectors in body to
% base)
% @param S the motion subspace between `body' and `base', i.e. the mapping
% from the vector of joint velocities describing the motion of `body' with
% respect to `base' to the twist of `body' with respect to `base',
% expressed in `body' frame
% @param qdotToV matrix describing the mapping from the derivative of the
% coordinate vector, qdot to the velocity vector, v.
%
% @retval ret the gradient of T with respect to q

qdot_to_twist = S * qdot_to_v;

nq = size(qdot_to_v, 2);
ret = zeros(numel(T), nq) * T(1); % for TaylorVar

R = T(1:3, 1:3);
Rx = R(:, 1);
Ry = R(:, 2);
Rz = R(:, 3);

qdot_to_omega_x = qdot_to_twist(1, :);
qdot_to_omega_y = qdot_to_twist(2, :);
qdot_to_omega_z = qdot_to_twist(3, :);

ret(1:3, :) = - Rz * qdot_to_omega_y + Ry * qdot_to_omega_z;
ret(5:7, :) = Rz * qdot_to_omega_x - Rx * qdot_to_omega_z;
ret(9:11, :) = -Ry * qdot_to_omega_x + Rx * qdot_to_omega_y;
ret(13:15, :) = R * qdot_to_twist(4:6, :);

end