function omega = rpydot2angularvel(rpy,rpydot)

% converts time derivatives of rpy (rolldot, pitchdot, yawdot) into the
% angular velocity vector in base frame.  See eq. (5.41) in Craig05.

E = rpydot2angularvelMatrix(rpy);
omega = E*rpydot;