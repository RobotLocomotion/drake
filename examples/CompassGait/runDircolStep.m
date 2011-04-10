function runDircolStep
% from an initial balancing condition, take one step and return to the
% balance upright.

p = CompassGaitPlant();

x0 = zeros(4,1); tf0 = 1; xf = zeros(4,1);
utraj0 = {PPTrajectory(foh(linspace(0,tf0,15),randn(1,15))),PPTrajectory(foh(linspace(0,tf0,15),randn(1,15)))};

con.mode{1}.mode_num = 1;
con.mode{2}.mode_num = 2;

con.mode{1}.u.lb = p.umin;
con.mode{1}.u.ub = p.umax;
con.mode{1}.u0.lb = 0;
con.mode{1}.u0.ub = 0;
con.mode{1}.uf.lb = 0;
con.mode{1}.uf.ub = 0;
con.mode{2}.u.lb = p.umin;
con.mode{2}.u.ub = p.umax;
con.mode{2}.u0.lb = 0;
con.mode{2}.u0.ub = 0;
con.mode{2}.uf.lb = 0;
con.mode{2}.uf.ub = 0;

con.mode{1}.x0.lb = x0;
con.mode{1}.x0.ub = x0;
con.mode{1}.xf.lb = [pi/8;-inf;-inf;-inf];
con.mode{2}.xf.lb = xf;
con.mode{2}.xf.ub = xf;

con.mode{1}.T.lb = .1;   
con.mode{1}.T.ub = 2;
con.mode{2}.T.lb = .1;   
con.mode{2}.T.ub = 2;

options.method='dircol';
tic
%options.grad_test = true;
%options.trajectory_cost_fun = {inline('0','T','X','U'),@(t,x,u)postImpactTrajCost(t,x,u,p)};
[utraj,xtraj,info] = trajectoryOptimization(p,{@cost,@cost},{@finalcost,@finalcost},{x0, [-pi/4;pi/4;2.0;0]},utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

figure(1);
fnplt(utraj);
 
figure(2);
fnplt(xtraj,[2 4]);

figure(3); clf
fnplt(xtraj,[2 3]);
xlabel('theta_st');
ylabel('theta_sw');
% plot collision surface here
%gx = .6*[-1,1]; gy = -gx -2*p.gamma;
%hold on; plot(gx,gy,'g--','LineWidth',2);
%T=utraj.getBreaks(); T=T(T>=utraj.getEvents()); X=xtraj.eval(T); X=X(2:5,:); U=utraj.eval(T);
%postImpactTrajCost(T,X,U,p)

return;

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
        h=t;
        dh = [1,zeros(1,size(x,1))];
      end
      
      function J = postImpactTrajCost(T,X,U,p)
        % encourage post-impact trajectory to leave collision surface orthogonally
        t0=T(1); x0=X(:,1); u0=U(:,1);
        xdot0=p.modes{2}.dynamics(t0,x0,u0);
        dphidx = [1,1,0,0]; % gradient of foot collision guard 1 w/respect to x
        J=100*abs(dphidx*xdot0)./norm(dphidx)./norm(xdot0);
      end
      
