function [ xtraj,utraj,info ] = testMassSpringDamperTrajOpt( )

x0 = [1 1]';
xf = [0 0]';
tf0 = 3;

p = RigidBodyManipulator('MassSpringDamper.urdf');

N = 11;
prog = DircolTrajectoryOptimization(p,N,tf0);

prog = prog.addStateConstraint(ConstantConstraint(x0),1);
prog = prog.addStateConstraint(ConstantConstraint(xf),N);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

prog = prog.addTrajectoryDisplayFunction(@(t,x,u)plotDircolTraj(t,x,u,[1 2]));

info = 0;
for attempts=1:10,
  tic
  [xtraj,utraj,~,~,info] = prog.solveTraj(tf0);
  toc
  if info==1, break; end
end

% if (nargout<1)
%   v = p.constructVisualizer();
%   v.playback(xtraj, struct('slider',true));
% end

end

function [g,dg] = cost(dt,x,u)
  R = eye(size(u,1));
  g = u'*R*u;
  dg = [zeros(1, 1 + size(x,1)),2*u(:)'];
end
      
function [h,dh] = finalCost(t,x)
  h = t;
  dh = [1,zeros(1,size(x,1))];
end

function plotDircolTraj(t,x,u,plotdims)
  figure(25);
  h=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
  drawnow;
  delete(h);
end