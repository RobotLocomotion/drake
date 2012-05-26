function taylorTest

oldpath = addpath([getDrakePath(),'/examples/Pendulum']);

order=2;

p = PendulumPlant();
t0=0;
x0=randn(2,1);
u0=0;

gradTest(@p.dynamics,t0,x0,u0,struct('tol',0.01))

path(oldpath);
oldpath = addpath([conf.root,'/examples/Acrobot']);

p = AcrobotPlant();
x0=randn(4,1);

gradTest(@p.dynamics,t0,x0,u0,struct('tol',0.01))

path(oldpath);
