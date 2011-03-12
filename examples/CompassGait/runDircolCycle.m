function runDircolCycle
% find stable limit cycle 

p = CompassGaitPlant();

% numbers from the first step of the passive sim 
x0 = [0;0;2;-.4];  
t1 = .417; 
x1 = [-.326;.22;-.381;-1.1];
tf = .713;
xf = x0;
N=15;
utraj0 = {PPTrajectory(foh(linspace(0,t1,N),zeros(1,N))),PPTrajectory(foh(linspace(0,tf-t1,N),zeros(1,N)))};

con.mode{1}.mode_num = 1;
con.mode{2}.mode_num = 2;

con.mode{1}.u.lb = p.umin;
con.mode{1}.u.ub = p.umax;
con.mode{2}.u.lb = p.umin;
con.mode{2}.u.ub = p.umax;

% make sure I take a reasonable sized step:
con.mode{1}.x0.lb = [0;-inf;-inf;-inf];
con.mode{1}.x0.ub = [0;inf;inf;inf];
con.mode{1}.xf.lb = [.1;-inf;-inf;-inf];

con.periodic = true;

con.mode{1}.T.lb = .2;   
con.mode{1}.T.ub = .5;
con.mode{2}.T.lb = .2;   
con.mode{2}.T.ub = .5;

options.method='dircol';
options.xtape0='simulate';
tic
%options.grad_test = true;
[utraj,xtraj,info] = trajectoryOptimization(p,{@cost,@cost},{@finalcost,@finalcost},{x0, x1},utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

t = xtraj.getBreaks();
t = linspace(t(1),t(end),100);
x = xtraj.eval(t);
plot(t,x);
x(:,1)
x(:,end)

v = CompassGaitVisualizer(p);
v.playback_speed = .2;
playback(v,xtraj);

end



      function [g,dg] = cost(t,x,u);
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
      end
      
      function [h,dh] = finalcost(t,x)
        h=0;
        dh = [0,zeros(1,size(x,1))];
      end
      
