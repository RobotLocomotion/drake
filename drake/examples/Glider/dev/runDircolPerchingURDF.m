function [utraj,xtraj, v, p]=runDircolPerchingURDF(p, utraj0)

tf0 = 1;
N=41;
if (nargin<1)
  options.floating = true;
  p = RigidBodyManipulator('GliderBalanced.urdf', options);
end

%for the URDF, a negative pitch is pitched up.
%    [X  Y  Z  Roll Pitch Yaw El
x0 = [-3.3 0 .4 0 0 0 0  7 0 0 0 0 0 0]';
xf = [0 0 0 0 -pi/4 0 0  0 0 -1 0 0 0 0]';
el_lo_limit = -.9473;
el_up_limit = .4463;

prog = DircolTrajectoryOptimization(p,N,[.45 1.5]);

prog = prog.addStateConstraint(BoundingBoxConstraint([-inf,-inf,-inf,-inf,-pi/2,-inf,el_lo_limit, -inf,-inf,-inf,-inf,-inf,-inf,-inf]',[inf,inf,inf,inf,pi/2,inf,el_up_limit, inf,inf,inf,inf,inf,inf,inf]'),1:N);
prog = prog.addStateConstraint(ConstantConstraint(x0),1);
prog = prog.addStateConstraint(BoundingBoxConstraint(xf-[0.03,0.03,0.03,inf,pi/2,inf,inf, inf,inf,inf,inf,inf,inf,inf]',xf+[0.03,0.03,0.03,inf,pi/2,inf,inf, inf,inf,inf,inf,inf,inf,inf]'),N);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@(t,x)finalCost(t,x,xf));

prog = prog.addTrajectoryDisplayFunction(@plotDircolTraj);

%options.MajorOptimalityTolerance=.03;
%options.MinorOptimalityTolerance=.03;

if nargin>1
 traj_init.u = utraj0;
else
 traj_init = struct();
end
tic
[xtraj,utraj,z,F,info,infeasible_constraint_name] = prog.solveTraj(tf0,traj_init);
if (info~=1) infeasible_constraint_name, error('failed to find a trajectory'); end
toc



v = p.constructVisualizer();
v.axis= [-4 .5 -.5 1];
v.playback_speed = .2;
v.playback(xtraj, struct('slider',true));

end

      function [g,dg] = cost(dt,x,u)
        Q = zeros(14,14); Q(14,14) = .1;
        g = x'*Q*x;
        if (nargout>1)
          dg = [0,2*x'*Q,zeros(1,size(u,1))];
        end
      end
        
      function [h,dh] = finalCost(t,x,xd)
        xerr = x-xd;

        Qf = diag([10 10 10 0 20 0 0 1 1 1 0 1 0 0]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh = [0,2*xerr'*Qf];
        end
      end
      
      function plotDircolTraj(t,x,u)
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
      end
