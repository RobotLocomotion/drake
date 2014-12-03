function [omega,domega] = rpydot2angularvel(rpy,rpydot)
% Converts time derivatives of rpy (rolldot, pitchdot, yawdot) into the
% angular velocity vector in base frame.  See eq. (5.41) in Craig05.
%
% @param rpy [roll; pitch; yaw]
% @param rpydot time derivative of rpy
%
% @retval omega angular velocity vector in base frame
% @retval domega. A 4 x 6 matrix. The gradient of omega w.r.t [rpy;rpydot]

if(nargout <= 1)
  E = rpydot2angularvelMatrix(rpy);
  omega = E*rpydot;
else
  [E,dE] = rpydot2angularvelMatrix(rpy);
  omega = E*rpydot;
  domega = [matGradMult(dE,rpydot) E];
end
