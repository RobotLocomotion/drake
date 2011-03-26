function [utraj,xtraj]=runDircol(p);

if (nargin<1)
  p = CartPolePlant();
end

x0 = zeros(4,1); tf0 = 4; xf = [0;pi;0;0];
utraj0 = PPTrajectory(foh(linspace(0,tf0,41),randn(1,41)));

%con.u.lb = p.umin;
%con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
con.T.lb = 2;   
con.T.ub = 6;

options.method='dircol';
tic
%options.grad_test = true;
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

if (nargout>0) 
  return;
end

c = tvlqr(p,xtraj,utraj,eye(4),1,eye(4));

t = xtraj.getBreaks();
t = linspace(t(1),t(end),100);
x = xtraj.eval(t);
plot(x(2,:),x(4,:));

v = CartPoleVisualizer();
v.playback(xtraj);

end

      function [g,dg] = cost(t,x,u);
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
      end
      
      function [h,dh] = finalcost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
      end
