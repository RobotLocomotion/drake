function [ xtraj,utraj,info ] = testMassSpringDamperTrajOpt( )

x0 = [1 1]';
xf = [0 0]';
tf0 = 3;

p = RigidBodyManipulator('MassSpringDamper.urdf');

con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;

options.method='dircol';
options.grad_method = 'user';
options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,[1 2]);

info = 0;
while (info~=1)
  utraj0 = PPTrajectory(foh(linspace(0,tf0,11),randn(1,11)));  
  tic
  [utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
  toc
end

% if (nargout<1)
%   v = p.constructVisualizer();
%   v.playback(xtraj, struct('slider',true));
% end

if info~=1
  path(oldpath);
  error('SNOPT did not find a solution!');
end

end

function [g,dg] = cost(t,x,u)
  R = eye(size(u,1));
  g = u'*R*u;
  dg = [zeros(1, 1 + size(x,1)),2*u(:)'];
end
      
function [h,dh] = finalcost(t,x)
  h = t;
  dh = [1,zeros(1,size(x,1))];
end

function [J,dJ]=plotDircolTraj(t,x,u,plotdims)
  figure(25);
  h=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
  drawnow;
  delete(h);
  J=0;
  dJ=[0*t(:);0*x(:);0*u(:)]';
end