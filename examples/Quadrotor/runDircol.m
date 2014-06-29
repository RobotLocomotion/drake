function runDircol

% simple planning demo which takes the quadrotor from hover at x=0m to a new hover at
% x=2m with minimal thrust.

r = Quadrotor();

N = 11;
traj_opt = DircolTrajectoryOptimization(r,N,[.1 4]);

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.base_z = .5;
u0 = double(nominalThrust(r));

traj_opt = traj_opt.addStateConstraint(ConstantConstraint(double(x0)),1);
traj_opt = traj_opt.addInputConstraint(ConstantConstraint(u0),1);

xf = x0;                       % final conditions: translated in x
xf.base_x = 2;
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(double(xf)),N);
traj_opt = traj_opt.addInputConstraint(ConstantConstraint(u0),N);

traj_opt = traj_opt.addRunningCost(@cost);
traj_opt = traj_opt.addFinalCost(@finalCost);

tf0 = 2;                      % initial guess at duration 
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));

info=0;
while (info~=1)
  tic
  [xtraj,utraj,z,F,info] = traj_opt.solveTraj(tf0,traj_init);
  toc
end

if (nargout<1)
  v = constructVisualizer(r);
  v.playback(xtraj,struct('slider',true));
end

end

      function [g,dg] = cost(dt,x,u)
        R = eye(4);
        g = u'*R*u;
        %g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        %dg = zeros(1, 1 + size(x,1) + size(u,1));
      end
      
      function [h,dh] = finalCost(t,x)
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
