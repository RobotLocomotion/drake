function runDircol

p = SCSDoublePendPlant();

x0 = [pi;zeros(3,1)]; tf0 = 4; xf = zeros(4,1); 
N=21;
utraj0 = PPTrajectory(foh(linspace(0,tf0,N),[zeros(1,N);randn(1,N)]));

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

t = xtraj.getBreaks();
t = linspace(t(1),t(end),100);
x = xtraj.eval(t);
plot(x(1,:),x(2,:));

v = SCSDoublePendVisualizer(p.robotobj);
playback(v,xtraj);

end



      function [g,dg] = cost(t,x,u);
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Q = diag([10,10,1,1]);
        R = 100;
        g = sum((Q*xerr).*xerr + (R*u).*u,1);
        
        if (nargout>1)
          dgdt = 0;
          dgdx = 2*xerr'*Q;
          dgdu = 2*u'*R;
          dg = [dgdt,dgdx,dgdu];
        end
      end
      
      function [h,dh] = finalcost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Qf = 100*diag([10,10,1,1]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh = [0, 2*xerr'*Qf];
        end
      end
      
