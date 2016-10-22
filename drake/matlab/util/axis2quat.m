function q = axis2quat(a)
% convert from axis angle, a = (x,y,z,theta) to quaternion (w,x,y,z)

a(1:3) = a(1:3)/norm(a(1:3));
q = [cos(a(4)/2); a(1:3)*sin(a(4)/2)];
  