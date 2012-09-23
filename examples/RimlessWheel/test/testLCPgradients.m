function testLCPgradients

options.floating = true;
m = PlanarRigidBodyModel('../RimlessWheel.urdf',options);

p = TimeSteppingRigidBodyManipulator(m,.01);
x0 = p.manip.resolveConstraints([0;0;randn;5*rand;randn;5*rand]);
v = p.constructVisualizer();
v.draw(0,x0);

options.grad_method = {'user','numerical'};

% first just ask for the gradients (todo: add a specific check)
[xdn,df] = update(p,0,x0,[]);

options.input_name={'t','q1','q2','q3','qd1','qd2','qd3'};
options.output_name={'qn1','qn2','qn3','qdn1','qdn2','qdn3'};
options.output=false;
gradTest(p,0,x0,[],options);


end
