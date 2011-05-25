function [utraj,xtraj]=runDircol(p)
%clear all
if (nargin<1)
  p = PlanePlant();
end

% OKTOFAIL

x0 = [0; 0; 0; 0];
tf0 = .5;
xf = [0; 5; 0; 0];

% generate a random trajectory
utraj0 = PPTrajectory(foh(linspace(0,tf0,11),randn(1,11)));

%con.u.lb = p.umin;
%con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
con.T.lb = 0.1;   
con.T.ub = 1;

options.method='dircol';
tic
%options.grad_test = true;
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

%con.uf.lb = 0;
%con.uf.ub = 0;
%options.xtape='simulate';
%[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj,con,options);
%if (info~=1) error('failed to improve the trajectory'); end


if (nargout>0) 
  return;
end

c = tvlqr(p,xtraj,utraj,eye(4),eye(1),eye(4));

t = xtraj.getBreaks();
t = linspace(t(1),t(end),100);
x = xtraj.eval(t);
%plot(x(2,:),x(4,:));

v = PlaneVisualizer();
v.playback(xtraj);

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
