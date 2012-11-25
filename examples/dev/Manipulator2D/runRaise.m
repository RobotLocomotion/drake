function [xtraj,utraj,ltraj,v,p,info,trans_info]=runRaise(xtraj_init, utraj_init, ltraj_init, trans_info)
clear mex
% p = AcrobotPlant();
p = Gripper2D_gcs;
N = 30;
% q0 = [0;.2;0;pi/2;0;0;0];
% q0 = [0;.1;0;pi/2;-pi/2;0;0];
% q0 = [0;.1;0;pi/2;-pi/2;-pi/8;pi/8];

q0 = [0;.1;0;1.2;-1.15;-.2;-.3;.4];
x0 = [q0;zeros(8,1)];
% q0 = [0;.1;0;pi/2;-pi/2;0;-pi/8;0];
% q0 =[0;.1;0;1.3;-1.3;0;-pi/8;.5];
% q0 = [0;.1;0;pi;-pi/2;0;-pi/8;0];

% xf= [0;.6;0;pi/2;0;0;0;0;zeros(8,1)];
% xf = [0;.6;0;2.2;-2;0;0;0;zeros(8,1)];
% xf = [0;.6;0;2.25;-2;-.3;0;.5;zeros(8,1)];

xf = [0;.6;0;2.25;-2;-.3;-.4;.4;zeros(8,1)];
% xf = x0;

xf_err = [0;0;0;.02*ones(5,1);10*ones(8,1)];
xfmax = xf + xf_err;
xfmin = xf - xf_err;


% xfmax = [0;.6;inf(6,1);10*ones(8,1)];
% xfmin = [-inf;.6;-inf;pi/2;-inf(4,1);-10*ones(8,1)];

xmin = [-.1;-inf;-inf;1;-inf; -pi/4; -pi/2;-pi/4;-2*ones(8,1)];

xmax = [.1;inf;inf; pi; inf; pi/4; pi/4;pi/2;2*ones(8,1)];

xmin(8) = q0(8)-.02;
xmax(8) = q0(8)+.02;

if exist('xtraj_init')
  traj0.x = xtraj_init;
  traj0.lambda = ltraj_init;
  utraj0 = utraj_init;
%   options.time = xtraj_init.pp.breaks;
  options.time = linspace(xtraj_init.tspan(1),xtraj_init.tspan(2), N);
else
  
  tf0 = 2;
  utraj0 = PPTrajectory(foh(linspace(0,tf0,N),randn(5,N)));
  
  xvec = repmat(x0,1,N) + (xf-x0)*linspace(0,1,N);
  d = 10;
  traj0.x = PPTrajectory(foh(linspace(0,tf0,N),xvec));
%   traj0.lambda = PPTrajectory(foh(linspace(0,tf0,N),zeros(14,N)));
traj0.lambda = PPTrajectory(foh(linspace(0,tf0,N),[.1*rand(6,N);zeros(8,N)]));
% traj0.lambda = PPTrajectory(foh(linspace(0,tf0,N),repmat([1;1;0;1;0;1;1;1],1,N)));

  traj0.x = traj0.x.setOutputFrame(p.getStateFrame);
end

N1 = floor(N/2);
% con.clambdai.lb.i = N1:N;
% con.clambdai.lb.clambdai = repmat([-inf;-inf;-inf;.1;-inf;.1;-inf(8,1)],1,N-N1+1);

con.clambdai.ub.i = 1:N;
con.clambdai.ub.clambdai = repmat([inf(7,1);.2;inf(6,1)],1,N);

con.fixtime = 1;
con.u.lb = p.umin;
con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xfmin;%[qf;-inf(6,1)];%[-inf(3,1); pi/2; -inf(4,1)];
con.xf.ub = xfmax;%[qf;inf(6,1)];%[inf(3,1); pi/2; inf(4,1)];
con.T.lb = .1;   
con.T.ub = 6;
con.noflight = 1;

con.x.lb = xmin;
con.x.ub = xmax;

%start with 100,100
con.alphamult = .1;
con.betamult = .1;

options.traj0 = traj0;

% options.method='implicit_dircol';
% options.method='implicit_dirtran2';
options.method='implicitdirtran';
options.MinorFeasibilityTolerance = 1e-6;
options.MajorFeasibilityTolerance = 1e-6;
options.MajorOptimalityTolerance = 1e-5;
options.MajorIterationsLimit =100;
options.IterationsLimit = 100000;
options.VerifyLevel=0;
options.SuperbasicsLimit=3000;
snprint('snopt.out');

if nargin > 3
  options.grad_I = trans_info.grad_I;
  options.grad_skip = trans_info.grad_skip;
else
  options.trimgrad = 0;
end

if exist('traj0','var')
  options.xtape0 = 'tape';
else
  xtraj0 = 0;
end

% options.grad_method='numerical';

tic
%options.grad_test = true;
[utraj,xtraj,info,trans_info] = p.trajectoryOptimization(@cost,@finalcost,x0,utraj0,con,options);
info
ltraj = xtraj.ltraj;
xtraj = xtraj.xtraj;
% if (info~=1) error('failed to find a trajectory'); end
toc

% t = xtraj.getBreaks();
% t = linspace(t(1),t(end),100);
% x = xtraj.eval(t);
% plot(x(1,:),x(2,:));

v = p.constructVisualizer;
% playback(v,xtraj);

end



      function [g,dg] = cost(dt,x,u,sys);
%         Q = 10e3;
        Q = diag([0;0;0]);
        
      
        R = 1;
        g = dt*sum((R*u).*u,1) + dt*x(1:3)'*Q*x(1:3);
        dg = [g/dt, 2*dt*x(1:3)'*Q zeros(1,size(x,1)-3),2*dt*u'*R];
%         g_i = sum(abs(R*u.*x([11 10 12])));
%         g = g_i*dt;
%         dg = [g_i, zeros(1,9), R*dt*(sign(x(10:12)).*abs(u([2 1 3])))', R*dt*(sign(u).*abs(x([11 10 12])))'];
        
%         %Add a cost to dt variance
%         R_dt = 100000;
%         g = g + R_dt*dt^2;
%         dg(1) = dg(1) + 2*R_dt*dt;
        return;
      end
      
      function [h,dh] = finalcost(t,x)
        R = 0;
        h = R*t;
        dh = [R,zeros(1,size(x,1))];
        return;
      end
      
