function rpydot = angularvel2rpydot(rpy,omega)

% converts the angular velocity vector to the time derivatives of rpy
% (rolldot, pitchdot, yawdot).  See eq. (5.41) in Craig05.  
% Derivation in rpydot2angularvel.
      
rpydot = angularvel2rpydotMatrix(rpy) * omega;