function testLCPgradients

options.floating = true;
m = PlanarRigidBodyModel('../RimlessWheel.urdf',options);

p = TimeSteppingRigidBodyManipulator(m,.01);
x0 = p.manip.resolveConstraints([0;1+rand;randn;5*rand;randn;5*rand]);


options.grad_method = {'user','numerical'};

% first just ask for the gradients (todo: add a specific check)
[xdn,df] = update(p,0,x0,[]);



end
