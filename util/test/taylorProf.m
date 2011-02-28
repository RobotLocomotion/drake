
load robotlib_config;

oldpath = addpath([conf.root,'/examples/Acrobot']);
p = AcrobotPlant();
t0=0;
u0=0;
fun=@(t,x,u)dynamics(p,t,x,u);
order=1;

profile on;
for i=1:100
  x0=randn(4,1);
  [xdot_taylor,dxdot_taylor]=taylor(fun,order,t0,x0,u0);
end
profile off;
profile viewer;

% NOTEST