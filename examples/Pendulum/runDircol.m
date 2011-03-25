function [utraj,xtraj]=runDircol(p);

if (nargin<1)
  p = PendulumPlant();
end

x0 = [0;0]; tf0 = 4; xf = [pi;0];
utraj0 = PPTrajectory(foh(linspace(0,tf0,21),randn(1,21)));

con.u.lb = p.umin;
con.u.ub = p.umax;
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

c = tvlqr(p,xtraj,utraj,eye(2),1,eye(2));

t = xtraj.getBreaks();
t = linspace(t(1),t(end),100);
x = xtraj.eval(t);
plot(x(1,:),x(2,:));

v = PendulumVisualizer();
v.playback(xtraj);

end

      function [g,dg] = cost(t,x,u);
        xd = repmat([pi;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Q = diag([10,1]);
        R = 100;
        g = sum((Q*xerr).*xerr,1) + (R*u).*u;
        
        if (nargout>1)
          dgdt = 0;
          dgdx = 2*xerr'*Q;
          dgdu = 2*u'*R;
          dg = [dgdt,dgdx,dgdu];
        end
      end
      
      function [h,dh] = finalcost(t,x)
        xd = repmat([pi;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Qf = 100*diag([10,1]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh = [0, 2*xerr'*Qf];
        end
      end
