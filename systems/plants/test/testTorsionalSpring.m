function testMassSpringDamper 

r = RigidBodyManipulator('torsional_spring_test.URDF');

% note: these have to match the urdf
m = 1;
l = 1;
I = 1;
b = 1;
k = 1;
g = 9.82;
rest_angle = pi/2;

for i=1:100
  x = randn(2,1);
  xdot = dynamics(r,0,x,[]);
  theta_ddot_desired = -b*x(2) - m*g*l*sin(x(1)) - k*(rest_angle - x(1));
  valuecheck(xdot(2),theta_ddot_desired);
end

