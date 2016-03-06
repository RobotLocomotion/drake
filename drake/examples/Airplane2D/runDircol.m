function [utraj,xtraj]=runDircol(p)

if (nargin<1)
  p = PlanePlant();
end

x0 = Point(getStateFrame(p));  % initial conditions: all-zeros
xf = x0;                       % final conditions: translated in y and rotated
xf.y = 8;
xf.theta = .1;
tf0 = .8;                      % initial guess at duration 
% remember that the model has a fixed forward speed of p.v (default is 10 m/s)

x0 = double(x0);
xf = double(xf);

N = 11;
prog = DircolTrajectoryOptimization(p,N,[0.1 1]);

prog = addStateConstraint(prog,ConstantConstraint(x0),1);
prog = addStateConstraint(prog,ConstantConstraint(xf),N);

prog = addRunningCost(prog,@cost);
prog = addFinalCost(prog,@finalCost);

prog = addTrajectoryDisplayFunction(prog,@(dt,x,u)plotDircolTraj(dt,x,u,[1 2]));
%options.MajorOptimalityTolerance=1e-2;

t_init = linspace(0,tf0,N);
info = 0;
while (info~=1)
  % generate a random trajectory
  traj_init.u = setOutputFrame(PPTrajectory(foh(t_init,randn(1,N))),getInputFrame(p));

  tic
  [xtraj,utraj,~,~,info]=solveTraj(prog,t_init,traj_init);
  toc
end

if (nargout<1)
  v = PlaneVisualizer(p);
  v.playback(xtraj);
end

end

      function [g,dg] = cost(dt,x,u)
        R = 0;
        g = u'*R*u;
        %g = sum((R*u).*u,1);
        %dg = [zeros(1,1+size(x,1)),2*u'*R];
        dg = zeros(1, 1 + size(x,1) + size(u,1));
      end
      
      function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
      end

      function [J,dJ]=plotDircolTraj(t,x,u,plotdims)
        figure(25);
        h=plot(x(plotdims(1),:),x(plotdims(2),:),'r.-');
        axis([-2 2 -1 10]); axis equal;
        drawnow;
        delete(h);
        J=0;
        dJ=[0*t(:);0*x(:);0*u(:)]';
      end
