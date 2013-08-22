function [utraj,xtraj]=runDircol(p);


if (nargin<1)
  p = GliderPlant();
end

x0 = getInitialState(p); tf0 = 1; xf = p.xd;
N=3; %=21;
utraj0 = PPTrajectory(foh(linspace(0,tf0,N),0*randn(1,N)));

con.u.lb = p.umin;
con.u.ub = p.umax;
con.x.lb = [-inf,-inf,-pi/2,p.phi_lo_limit,-inf,-inf,-inf]';
con.x.ub = [inf,inf,pi/2,p.phi_up_limit,inf,inf,inf]';
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf-[ 0, 0, inf, inf, inf, inf, inf]';
con.xf.ub = xf+[ 0, 0, pi/2, inf, inf, inf, inf]';
con.T.lb = .5;   
con.T.ub = 1.5;

options.method='dircol';
options.grad_method={'user'};%,'taylorvar'};
tic
%options.grad_test = true;
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@(t,x)finalcost(t,x,xf),x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

if (nargout>0) 
  return;
end

v = GliderVisualizer(p);
v.playback(xtraj);

end

      function [g,dg] = cost(t,x,u);
        R = .1;
        g = u'*R*u;
        if (nargout>1)
          dg = [0,zeros(1,length(x)),2*u'*R];
        end
      end
        
      function [h,dh] = finalcost(t,x,xd)
        xerr = x-xd;

        Qf = diag([10 10 10 0 1 1 1]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh = [0,2*xerr'*Qf];
        end
      end
