function testKinematics

m = PlanarRigidBodyManipulator('../Acrobot.urdf');
m.body(1).contact_pts = randn(2,4);
m.body(2).contact_pts = randn(2,4);
m = compile(m);
options.grad_method = {'user','taylorvar'};

for i=1:100
  q = randn(2,1); 
  [x,J,dJ] = geval(1,@contactPositions,m,q,options);
end

end