function [utraj,xtraj, v, p]=runDircolPerchingURDF(p, utraj0)

tf0 = 1;
N=4; %=21;
if (nargin<1)
  options.floating = true;
  options.terrain = [];
  p = PlanarRigidBodyManipulator('GliderBalanced.urdf', options);
end
if (nargin<2)
    utraj0 = PPTrajectory(foh(linspace(0,tf0,N),0*randn(1,N)));
end
%for the URDF, a negative pitch is pitched up.
%    [X    Z  Theta Phi Xdot Zdot
x0 = [-3.3 .4 0     0   7.0    0    0 0]';
xf = [0    0  -pi/4 0   0    -1   0 0]';
phi_lo_limit = -.9473;
phi_up_limit = .4463;

con.u.lb = p.umin;
con.u.ub = p.umax;
con.x.lb = [-inf,-inf,-pi/2,phi_lo_limit,-inf,-inf,-inf, -inf]';
con.x.ub = [inf,inf,pi/2,phi_up_limit,inf,inf,inf,inf]';
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf-[ 0.03, 0.03, pi/2, inf, inf, inf, inf, inf]';
con.xf.ub = xf+[ 0.03, 0.03, pi/2, inf, inf, inf, inf, inf]';
con.T.lb = .45;   
con.T.ub = 1.5;

options.MajorOptimalityTolerance=.03;
options.MinorOptimalityTolerance=.03;
options.method='dircol';
options.grad_method={'numerical'};%'user'};
options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u);  % for debugging
tic
%options.grad_test = true;
disp('Starting optimization');
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@(t,x)finalcost(t,x,xf),x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

v = p.constructVisualizer();
v.axis= [-4 .5 -.5 1];
v.playback_speed = .2;
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

        Qf = diag([10 10 20 0 1 1 1 0]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh = [0,2*xerr'*Qf];
        end
      end
      
      function [J,dJ]=plotDircolTraj(t,x,u)
      %{  
      figure(25);
        clf;
        %Determines length of lines representing pitch of the aircraft
        x_len = max(x(1,:))-min(x(1,:));
        x_len = x_len/50;
        z_len = max(x(2,:))-min(x(2,:));
        z_len = z_len/50;
        hold on
        for i = 1:size(x,2)
            xpt = x(1,i); zpt = x(2,i); pitch = -x(3,i);
            plot([xpt-x_len*cos(pitch) xpt+x_len*cos(pitch)], ...
                 [zpt-z_len*sin(pitch) zpt+z_len*sin(pitch)]);
        end
        h=plot(x(1,:),x(2,:),'r.-');
        drawnow;
        hold off;
        delete(h);
      %}
        J=0;
        dJ=[0*t(:);0*x(:);0*u(:)]';
      end
