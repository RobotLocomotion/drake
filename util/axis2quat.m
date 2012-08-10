function q = axis2quat(a)
% convert from axis angle, a = (x,y,z,theta) to quaternion (x,y,z,w)

q = [cos(a(4)/2); a(1:3)*sin(a(4)/2)];
  