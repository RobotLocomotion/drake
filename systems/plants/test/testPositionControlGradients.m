function testPositionControlGradients

options.enable_fastqp = false; % want to use only one solver for numerical gradient
p = TimeSteppingRigidBodyManipulator('ActuatedPendulum.urdf', .01,options);
p = enableIdealizedPositionControl(p, true);
p = compile(p);

num_tests = 10;

x = 100*rand(2,num_tests);
u = rand(1,num_tests);

for i=1:num_tests
    
    [xdn1,df1] = geval(1,@update,p,0,x(:,i),u(:,i),struct('grad_method','numerical'));
    [xdn2,df2] = geval(1,@update,p,0,x(:,i),u(:,i),struct('grad_method','user'));

    valuecheck(xdn2,xdn1,1e-4);
    valuecheck(df2,df1,1e-4);
    
end