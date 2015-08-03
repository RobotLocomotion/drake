function rpydot = angularvel2rpydot(rpy,omega)
% Converts the angular velocity vector to the time derivatives of rpy
% (rolldot, pitchdot, yawdot).  See eq. (5.41) in Craig05.  
% Derivation in rpydot2angularvel.
%
% @param rpy [roll; pitch; yaw]
% @param omega angular velocity vector in world frame
%
% @retval rpydot time derivative of rpy
      
rpydot = angularvel2rpydotMatrix(rpy) * omega;