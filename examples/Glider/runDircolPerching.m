function [utraj,xtraj]=runDircolPerching(p)


if (nargin<1)
    close all
    megaclear;
  p = GliderPlant();
end

x0 = getInitialState(p);
x0 = [-3.5 0.1 0 0 7 0 0]';
tf0 = 1.0;
xf = p.xd;
N=41;
utraj0 = PPTrajectory(foh(linspace(0,tf0,N),0*randn(1,N)));

con.u.lb = p.umin;
con.u.ub = p.umax;
con.x.lb = [-4,-1,-pi/2,p.phi_lo_limit,-inf,-inf,-inf]';
con.x.ub = [1,1,pi/2,p.phi_up_limit,inf,inf,inf]';
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = [ 0, 0, pi/6 -inf, -2, -2, -inf]';
con.xf.ub = [ 0, 0, 1, inf, 2, 2, inf]';
con.T.lb = 0;
con.T.ub = 2;

options.method='dircol';
options.grad_method={'user'};%,'taylorvar'};
%options.xtape0='simulate';
options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,options);
tic
%options.grad_test = true;
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@(t,x)finalcost(t,x,xf),x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

if (nargout>0)
  return;
end

figure(1)
fnplt(xtraj)

figure(2)
fnplt(utraj)

fnplt(xtraj,4)

fnplt(xtraj,3)

save('glider_trajs','xtraj','utraj');

v = GliderVisualizer(p);
v.playback(xtraj);

end

      function [g,dg] = cost(t,x,u);
        R = 100;
        g = u'*R*u;
        if (nargout>1)
          dg = [0,zeros(1,length(x)),2*u'*R];
        end
      end

      function [h,dh] = finalcost(t,x,xd)
        xerr = x-xd;

        Qf = diag([10 10 1 10 1 1 1]);
        h = sum((Qf*xerr).*xerr,1);

        if (nargout>1)
          dh = [0,2*xerr'*Qf];
        end
      end

      function [J,dJ]=plotDircolTraj(t,x,u,options)

%plotdims=options.plotdims;

  figure(1)
  clf;
  hold on
  plot(x(1,:),x(2,:),'.-');
  if(isfield(options,'xtraj0'))
       N=length(options.xtraj0);
       for k=1:N
       fnplt(options.xtraj0{k});
       end
  end
  hold off
  axis([-4 1 -1 1]);
  drawnow;
  %delete(h);
  J=0;
  dJ=[0*t(:);0*x(:);0*u(:)]';
end
