function a = rpy2axis(rpy)

% converts roll, pitch, yaw notation (e.g., as used in URDF) to axis angle
% notation (e.g. as used by VRML)
% from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToAngle/index.htm

typecheck(rpy,'numeric');
sizecheck(rpy,3);

c = cos(rpy); s = sin(rpy);

a(4,1) = 2 * acos(prod(c) - prod(s));    % angle
a(1) = s(1)*s(2)*c(3) + c(1)*c(2)*s(3);  % x
a(2) = s(1)*c(2)*c(3) + c(1)*s(2)*s(3);  % y
a(3) = c(1)*s(2)*c(3) - s(1)*c(2)*s(3);  % z

a(1:3)=a(1:3)./(norm(a(1:3))+eps); % normalize

