function dX = dXpln( theta, r, varIndex)

% dXpln  coordinate transform derivative wrt theta for planar vectors.
% dXpln(theta,r) calculates the derivative of the coordinate transform 
% matrix from A to B coordinates for planar motion vectors, where 
% coordinate frame B is located at point r (2D vector expressed in A 
% coords) relative to frame A, and is rotated by an angle theta (radians) 
% relative to A.

c = cos(theta);
s = sin(theta);
if varIndex == 1
dX = [0 0 0; c*r(1)+s*r(2) -s c; -s*r(1)+c*r(2) -c -s];
elseif varIndex == 2
  dX = [0 0  0; s 0 0; c 0 0];
elseif varIndex == 3
  dX = [0 0 0; -c 0 0; s 0 0];
end