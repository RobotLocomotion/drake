function runDircol

r = Quadrotor();

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.base_z = .5;
u0 = nominalThrust(r);

xf = x0;                       % final conditions: translated in x
xf.base_x = 2;
uf = u0;

tf0 = 2;                      % initial guess at duration 

x0 = double(x0);
u0 = double(u0);
xf = double(xf);
uf = double(uf);

con.u.lb = r.umin;
con.u.ub = r.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.u0.lb = u0;
con.u0.ub = u0;
con.xf.lb = xf;
con.xf.ub = xf;
con.uf.lb = uf;
con.uf.ub = uf;
con.T.lb = 0.1;   
con.T.ub = 4;

options.method='dircol';
options.MajorOptimalityTolerance=1e-2;
options.grad_method = 'taylorvar';
%options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,[1 2]);  % for debugging
%options.grad_test = true;

info = 0;
%while (info~=1)
  % generate a random trajectory
  utraj0 = PPTrajectory(foh(linspace(0,tf0,11),repmat(u0,1,11)+randn(4,11)));
  
  tic
  [utraj,xtraj,info] = trajectoryOptimization(r,@cost,@finalcost,x0,utraj0,con,options);
  toc
%end

if (nargout<1)
  v = constructVisualizer(r);
  v.playback(xtraj,struct('slider',true));
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
