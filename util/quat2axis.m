function a=quat2axis(q)
% converts unit quaternions to axis angle representation
% from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

typecheck(q,'numeric');
sizecheck(q,4);

s = sqrt(1-q(4)^2)+eps;
a = q(1:3)/s;
a(4) = 2*acos(q(4));



