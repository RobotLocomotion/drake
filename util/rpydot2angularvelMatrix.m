function E = rpydot2angularvelMatrix(rpy)

% matrix that converts time derivatives of rpy (rolldot, pitchdot, yawdot) into the
% angular velocity vector expressed in base frame.  See eq. (5.41) in Craig05.

% Derived using:
% syms r p y real; rpy=[r p y];
% R = rpy2rotmat(rpy);
% E(1,:) = jacobian(R(3,1),rpy)*R(2,1) + jacobian(R(3,2),rpy)*R(2,2) + jacobian(R(3,3),rpy)*R(2,3);
% E(2,:) = jacobian(R(1,1),rpy)*R(3,1) + jacobian(R(1,2),rpy)*R(3,2) + jacobian(R(1,3),rpy)*R(3,3);
% E(3,:) = jacobian(R(2,1),rpy)*R(1,1) + jacobian(R(2,2),rpy)*R(1,2) + jacobian(R(2,3),rpy)*R(1,3); 
% simplify(E)
% Note: I confirmed that the same recipe yields (5.42)

% r=rpy(1);
p=rpy(2);
y=rpy(3);

E = [ cos(p)*cos(y), -sin(y), 0; ...
      cos(p)*sin(y),  cos(y), 0; ...
            -sin(p),       0, 1];