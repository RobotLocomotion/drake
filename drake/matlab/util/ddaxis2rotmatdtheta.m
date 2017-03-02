function ddRdtheta = ddaxis2rotmatdtheta(a)
% convert from axis angle, a = [x;y;z;theta];
a(1:3) = a(1:3)/norm(a(1:3));
x = a(1);
y = a(2);
z = a(3);
theta = a(4);
c = 1-cos(theta);
dc = sin(theta);
ddc = cos(theta);
ddRdtheta = [-cos(theta)+x^2*ddc x*y*ddc+z*sin(theta) x*z*ddc-y*sin(theta);...
  y*x*ddc-z*sin(theta) -cos(theta)+y^2*ddc y*z*ddc+x*sin(theta);...
  z*x*ddc+y*sin(theta) z*y*ddc-x*sin(theta) -cos(theta)+z^2*ddc];
end