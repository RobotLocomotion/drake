function dRdtheta = daxis2rotmatdtheta(a)
% convert from axis angle, a = [x;y;z;theta];
a(1:3) = a(1:3)/norm(a(1:3));
x = a(1);
y = a(2);
z = a(3);
theta = a(4);
c = 1-cos(theta);
dc = sin(theta);
dRdtheta = [-sin(theta)+x^2*dc x*y*dc-z*cos(theta) x*z*dc+y*cos(theta);...
  y*x*dc+z*cos(theta) -sin(theta)+y^2*dc y*z*dc-x*cos(theta);...
  z*x*dc-y*cos(theta) z*y*dc+x*cos(theta) -sin(theta)+z^2*dc];
end