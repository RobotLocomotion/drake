function testKinematics

m = PlanarRigidBodyModel('../Acrobot.urdf');
options.grad_method = {'user','taylorvar'};

for i=1:100
  q = randn(2,1); 
  [x,J,dJ] = geval(1,@kinTest,m,q,options);
end

end