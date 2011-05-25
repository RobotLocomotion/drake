function [utraj,xtraj]=runDircol(p);

if (nargin<1)
  p = PendulumPlant();
end

x0 = [0;0]; tf0 = 4; xf = [pi;0];
utraj0 = PPTrajectory(foh(linspace(0,tf0,21),randn(1,21)));

con.u.lb = p.umin;
con.u.ub = p.umax;
%con.u0.lb = 0;
%con.u0.ub = 0;
%con.uf.lb = 0;
%con.uf.ub = 0;
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
 % OKTOFAIL

toc

if (nargout>0) 
  return;
end

v = PendulumVisualizer();
v.playback(xtraj);

end

      function [g,dg] = cost(t,x,u);
        R = 10;
        g = (R*u).*u;
        
        if (nargout>1)
          dg = [zeros(1,3),2*u'*R];
        end
      end
      
      function [h,dh] = finalcost(t,x)
        h = t;
        if (nargout>1)
          dh = [1, zeros(1,2)];
        end
      end
      
