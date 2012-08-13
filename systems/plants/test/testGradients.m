function testGradients

p = PlanarRigidBodyManipulator('../../../examples/Acrobot/Acrobot.urdf');
options.grad_method = {'user','taylorvar'};

for i=1:100
  t = rand;
  x = randn(4,1);
  u = randn;
  
  [xdot,dxdot] = geval(@p.dynamics,t,x,u,options);
end