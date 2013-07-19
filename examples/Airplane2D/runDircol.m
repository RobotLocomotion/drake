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

con.u.lb = p.umin;
con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
con.T.lb = 0.1;   
con.T.ub = 1;

options.method='dircol';
options.MajorOptimalityTolerance=1e-2;
options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,[1 2]);  % for debugging
%options.grad_test = true;

info = 0;
while (info~=1)
  % generate a random trajectory
  utraj0 = PPTrajectory(foh(linspace(0,tf0,11),randn(1,11)));
  
  tic
  [utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
  toc
end

if (nargout<1)
  v = PlaneVisualizer(p);
  v.playback(xtraj);
end

end

      function [g,dg] = cost(t,x,u)
        R = 0;
        g = u'*R*u;
        %g = sum((R*u).*u,1);
        %dg = [zeros(1,1+size(x,1)),2*u'*R];
        dg = zeros(1, 1 + size(x,1) + size(u,1));
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
