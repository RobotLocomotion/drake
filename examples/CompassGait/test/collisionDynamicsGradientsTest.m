function collisionDynamicsGradientsTest()
% Tests user gradients vs TaylorVar gradients to check
% consistency (e.g., should break if you update
% the original function you autogenerated gradients for but
% forgot to re-generate the gradients.

oldpath=addpath('..');
p=CompassGaitPlant();


for i=1:20

  
t=rand;
m=1;
xm=randn(4,1);
u=randn;
%xm = [0.3927; -0.0907; 0.0144; 0.1396];
  


[xp,status,dxp]=geval(3,@p.collisionDynamics,m,t,xm,u,struct('grad_method','user'));

[xpt,statust,dxpt]=geval(3,@p.collisionDynamics,m,t,xm,u,struct('grad_method','taylorvar'));

if (any(abs(xp-xpt)>1e-12))
  path(oldpath);
  error('dynamics don''t match!');
end
if (any(abs(statust-status))>1e-12)
  path(oldpath);
  error('status doesn''t match!');
end
if (any(abs(dxp(:)-dxpt(:))>1e-12))
  path(oldpath);
  error('gradients don''t match!');
end
  
end

path(oldpath);
