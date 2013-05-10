function rpydot = angularvel2rpydot(rpy,omega)

% converts the angular velocity vector to the time derivatives of rpy
% (rolldot, pitchdot, yawdot).  See eq. (5.41) in Craig05.  
% Derivation in rpydot2angularvel.

r=rpy(1);p=rpy(2);y=rpy(3);

% warning: note the singularities!
Einv = [ cos(y)/cos(p), sin(y)/cos(p), 0; ...
               -sin(y),        cos(y), 0; ...
         cos(y)*tan(p), tan(p)*sin(y), 1];
       
rpydot = Einv*omega;