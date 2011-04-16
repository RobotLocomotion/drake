function c=PendulumLQRTree(p)

if (nargin<1)
  p=PendulumPlant();
end

xG=[pi;0];
uG=0;
Q = diag([10 1]);
R = 1;

options.num_branches=5;
%options.verify=false;
options.xs = [0;0]; 
options.monom_order=4;
c = LQRTree.buildLQRTree(p,xG,uG,@()rand(2,1).*[2*pi;10]-[pi;5],Q,R,options);
