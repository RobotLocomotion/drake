function taylorTest

oldpath = addpath([getDrakePath(),'/examples/Pendulum']);

order=2;

p = PendulumPlant();
t0=0;
x0=randn(2,1);
u0=0;

options.grad_method = {'user','taylorvar'};
[f,df]=geval(@p.dynamics,t0,x0,u0,options);

%gradTest(@p.dynamics,t0,x0,u0,struct('tol',0.01))

path(oldpath);
oldpath = addpath([getDrakePath(),'/examples/Acrobot']);

p = AcrobotPlant();
x0=randn(4,1);

[f,df]=geval(@p.dynamics,t0,x0,u0,options);
%gradTest(@p.dynamics,t0,x0,u0,struct('tol',0.01))

path(oldpath);
