function [utraj,xtraj,field]=runDircolWithObs(p)
%clear all
if (nargin<1)
  p = PlanePlant();
end

x0 = [3.9; 0; 0; 0];
tf0 = .7;
xf = [5; 9; 0; 0];

utraj0 = PPTrajectory(foh(linspace(0,tf0,11),0*randn(1,11)));
utraj0 = setOutputFrame(utraj0,p.getInputFrame);

%con.u.lb = p.umin;
%con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf; %[xf(1:2);-inf(2,1)];
con.xf.ub = xf; %[xf(1:2);inf(2,1)];
con.T.lb = .2;
con.T.ub = 3;


% add obstacles
disp('Adding obstacles...');
field = ObstacleField();
field = field.GenerateRandomObstacles();
con = field.AddConstraints(con);

figure(25); clf;  hold on;
v = PlaneVisualizer(p,field);
v.draw(0,x0);
drawnow

disp('Running trajectory optimization...');

options.method='dircol';
options.MajorOptimalityTolerance=1e-2;
options.trajectory_cost_fun=@(t,x,u)plotDircolTraj(t,x,u,[1 2]);  % for debugging
%options.grad_method={'user','taylorvar'};
tic
%options.grad_test = true;
[utraj,xtraj,info] = trajectoryOptimization(p,@(t,x,u)cost(t,x,u,field),@finalcost,x0,utraj0,con,options);
if (info~=1) error(['Airplane2D:SNOPT:INFO',num2str(info)],'failed to find a trajectory'); end
toc

% upsample
ts = linspace(utraj.tspan(1),utraj.tspan(2),41);
options.xtape0='simulate';
utraj0 = PPTrajectory(foh(ts,utraj.eval(ts))); utraj0 = setOutputFrame(utraj0,utraj.getOutputFrame);
[utraj,xtraj,info] = trajectoryOptimization(p,@(t,x,u)cost(t,x,u,field),@finalcost,x0,utraj0,con,options);
if (info~=1) error(['Airplane2D:SNOPT:INFO',num2str(info)],'failed to improve the trajectory'); end

hold off;

if (nargout>0) 
  return;
end

%disp('Stabilizing trajectory...');
c = tvlqr(p,xtraj,utraj,eye(4),1,eye(4));

disp('Visualizing trajectory...');
v.playback(xtraj);
%v.display_dt=0;v.playback_speed=.5; v.playbackAVI(xtraj,'PlaneWithObs');

disp('Done.');

end

      function [g,dg] = cost(t,x,u,field)
        R = 0.0001;
        
        % minimize the max obstacle constraint
        [c,dc] = field.obstacleConstraint(x);
        [c,i]=max(c);
        dc = dc(i,:);
        
        g = c + u'*R*u;
        %g = sum((R*u).*u,1);
        dg = [0,dc,2*u'*R];
        
        %dg = zeros(1, 1 + size(x,1) + size(u,1));
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
