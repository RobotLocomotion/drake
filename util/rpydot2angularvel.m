function omega = rpydot2angularvel(rpy,rpydot)
% Converts time derivatives of rpy (rolldot, pitchdot, yawdot) into the
% angular velocity vector in base frame.  See eq. (5.41) in Craig05.
%
% @param rpy [roll; pitch; yaw]
% @param rpydot time derivative of rpy
%
% @retval omega angular velocity vector in base frame

E = rpydot2angularvelMatrix(rpy);
omega = E*rpydot;