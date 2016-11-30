function testTorsionalSpring 

r = RigidBodyManipulator('TorsionalSpring.urdf');

% note: these have to match the urdf
m = 1;
l = 1;
I = m*l^2;
k = 10;
b = 1;
g = 9.81;
rest_angle = pi/2;


for i=1:100
  x = randn(2,1);
  xdot = dynamics(r,0,x,[]);
  theta_ddot_desired = (-b*x(2) -m*g*l*sin(x(1)) + k*(rest_angle - x(1)))/I;
  valuecheck(xdot(2),theta_ddot_desired);
end

torsional_spring = r.force{1};
q = getRandomConfiguration(r);
qd = rand(r.getNumVelocities());

geval_options.grad_method = {'user', 'taylorvar'};
[~, ~] = geval(1, @(q, qd) torsional_spring.computeSpatialForce(r, q, qd), q, qd, geval_options);

end
