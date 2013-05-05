function a=quat2axis(q)
% converts unit quaternions to axis angle representation
% from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

typecheck(q,'numeric');
sizecheck(q,4);

q = q/norm(q);
s = sqrt(1-q(1)^2)+eps;
a = q(2:4)/s;
a(4) = 2*acos(q(1));



