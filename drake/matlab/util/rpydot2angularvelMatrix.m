function [E,dE] = rpydot2angularvelMatrix(rpy)
% Computes matrix that converts time derivatives of rpy 
% (rolldot, pitchdot, yawdot) into the angular velocity vector expressed in
% base frame.  See eq. (5.41) in Craig05.
%
% @param rpy [roll; pitch; yaw]
%
% @retval E matrix such that omega = E * rpyd, where omega is the angular
% velocity vector in base frame and rpyd is the time derivative of rpy
% @retval dE. A 9 x 3 matrix. The gradient of E w.r.t rpy

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

cos_p = cos(p);
sin_p = sin(p);
cos_y = cos(y);
sin_y = sin(y);

E = [ cos_p*cos_y, -sin_y, 0;...
      cos_p*sin_y,  cos_y, 0;...
	   -sin_p,      0, 1];
if(nargout>1)
  rows = zeros(7,1);
  cols = zeros(7,1);
  vals = zeros(7,1);
  rows(1) = 1; cols(1) = 2; vals(1) = -sin_p*cos_y;
  rows(2) = 1; cols(2) = 3; vals(2) = -cos_p*sin_y;
  rows(3) = 2; cols(3) = 2; vals(3) = -sin_p*sin_y;	
  rows(4) = 2; cols(4) = 3; vals(4) = cos_p*cos_y;
  rows(5) = 3; cols(5) = 2; vals(5) = -cos_p;
  rows(6) = 4; cols(6) = 3; vals(6) = -cos_y;
  rows(7) = 5; cols(7) = 3; vals(7) = -sin_y;
  dE = sparse(rows,cols,vals,9,3,7);
end
