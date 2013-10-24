function testKinematics

m = PlanarRigidBodyManipulator('../Acrobot.urdf');
b = getBody(m,1);
b.contact_pts = randn(2,4);
m = setBody(m,1,b);
b = getBody(m,2);
b.contact_pts = randn(2,4);
m = setBody(m,2,b);
m = compile(m);
options.grad_method = {'user','taylorvar'};

for i=1:100
  q = randn(2,1); 
  [x,J] = geval(1,@contactPositions,m,q,options);
end

end