function [utraj, xtraj] = runDircol_transverse

p = PendulumPlant();

x0 = [0;0]; tf0 = 4; xf = [pi;0];
utraj0 = PPTrajectory(spline(linspace(0,tf0,11),randn(1,11)));

con.u.lb = p.umin;
con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
con.T.lb = 2;   
con.T.ub = 6;

options=struct();
tic

disp('Searching for trajectory')
%options.grad_test = true;
options.method = 'dircol';
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

xi = [-0.2;-0.1];


t = xtraj.getBreaks();
t = linspace(t(1),t(end),100);
x = xtraj.eval(t);
plot(x(1,:),x(2,:));

v = PendulumVisualizer();
v.playback(xtraj);

%disp('Computing tvlqr...')
%cfull = tvlqr(p,xtraj,utraj,eye(2),1,1*eye(2));
%disp('Simulating...')
%sysfull = cascade(feedback(p,cfull),v);
%simulate(sysfull,xtraj.tspan*0.99,xi);


w= randn(2,1);
Nsteps = 50;

% Use orthogonal initial and final surface normals (not essential)
fs0 = p.dynamics(0,xtraj.eval(0),utraj.eval(0));
fsend = p.dynamics(utraj.tspan(end),xtraj.eval(utraj.tspan(end)),utraj.eval(utraj.tspan(end)));
init_surf_normal = fs0/norm(fs0);
final_surf_normal = fsend/norm(fsend);

disp('Designing transversal surfaces...')
TransSurf = TransversalSurface.design(p,w, xtraj, utraj, Nsteps,1,init_surf_normal, final_surf_normal);

disp('Computing transverse lqr...')
ctrans = transverseTVLQR(p,xtraj,utraj,10*eye(2),1,10*eye(2),TransSurf);

sys = cascade(feedback(p,ctrans),v);
disp('Simulating...')
simulate(sys,xtraj.tspan*0.99,xi);



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
