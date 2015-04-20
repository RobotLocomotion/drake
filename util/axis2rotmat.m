function R = axis2rotmat(a)
% convert from axis angle, a = [x;y;z;theta];
if isnumeric(a) % for TrigPoly
  a(1:3) = a(1:3)/norm(a(1:3));
end
x = a(1);
y = a(2);
z = a(3);
theta = a(4);
c = 1-cos(theta);
R = [cos(theta)+x^2*c x*y*c-z*sin(theta) x*z*c+y*sin(theta);...
  y*x*c+z*sin(theta) cos(theta)+y^2*c y*z*c-x*sin(theta);...
  z*x*c-y*sin(theta) z*y*c+x*sin(theta) cos(theta)+z^2*c];
end