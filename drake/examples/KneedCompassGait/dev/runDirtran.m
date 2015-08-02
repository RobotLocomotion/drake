function runDirtran

% run dirtran multiple times, reducing slack variable tolerance to zero:
[xtraj,utraj,ltraj,v,p]=runDirtranIteration;
[xtraj,utraj,ltraj,v,p]=runDirtranIteration(xtraj, utraj, ltraj,.1,.1);
[xtraj,utraj,ltraj,v,p]=runDirtranIteration(xtraj, utraj, ltraj,.01,.01);
[xtraj,utraj,ltraj,v,p]=runDirtranIteration(xtraj, utraj, ltraj,1e-3,1e-3);
[xtraj,utraj,ltraj,v,p]=runDirtranIteration(xtraj, utraj, ltraj,0,0,1000);
%v.playback(xtraj,struct('slider',true));
v.playback(xtraj);

end


function [xtraj,utraj,ltraj,v,p]=runDirtranIteration(xtraj_init, utraj_init, ltraj_init,alphamult,betamult,maxIter)

options.floating = true;
p = PlanarRigidBodyManipulator('KneedCompassGait.urdf',options);
N = 30;
q0 = [0; 1; 0; 0;0;0;];
% q0 = [0;cos(pi/16);pi/16;-pi/8;0*pi/8;-pi/8];
% qf = [sin(pi/8); cos(pi/8); -pi/8; 0; pi/4; 0];

% state order:
% x
% z
% pitch?
% red knee?
% hip?
% blue knee

con.linperiodic = zeros(12);
con.linperiodic(2,2) = 1; %z 
con.linperiodic(3,3) = 1; %pitch-hip w/symmetry
con.linperiodic(3,5) = 1; %pitch-hip w/symmetry
con.linperiodic(4,6) = 1; %knee w/symmetry
con.linperiodic(6,4) = 1; %knee w/symmetry
con.linperiodic(5,5) = -1; %hip w/symmetry

con.linperiodic(7,7) = 1; %x-vel
con.linperiodic(8,8) = 1; %z-vel
con.linperiodic(9,9) = 1; %pitch-hip w/symmetry
con.linperiodic(9,11) = 1; %pitch-hip w/symmetry
con.linperiodic(10,12) = 1; %knee w/symmetry
con.linperiodic(12,10) = 1; %knee w/symmetry
con.linperiodic(11,11) = -1; %hip w/symmetry

%CHANGED Q0, XFMAX, XFMIN, X.UB

%read to blue
% qf = [.3;1;0;0;0;0];
x0 = [q0;zeros(6,1)];
xf = x0 + [.6;zeros(11,1)];
% xfmax = [inf;inf;pi/16;-pi/8;0;-pi/8;inf(6,1)];
% xfmin = [.5;0;pi/16;-pi/8;0;-pi/8;-inf(6,1)];
% xfmax = [inf;inf;0;0;0;0;inf(6,1)];
% xfmin = [.5;0;0;0;0;0;-inf(6,1)];

% xfmax = [inf;1;zeros(4,1);inf;0;inf(4,1)];
% xfmin = [.5;1;zeros(4,1);-inf;0;-inf(4,1)];
% 
% xfmax = [inf;x0(2:end)];
% xfmin = [.5;x0(2:end)];

% xfmax = [inf;x0(2:6);inf;0;inf(4,1)];
% xfmin = [.5;x0(2:6);-inf;0;-inf(4,1)];

xfmax = [inf(12,1)];
xfmin = [.4;-inf(11,1)];

if exist('xtraj_init')
  traj0.x = xtraj_init;
  traj0.lambda = ltraj_init;
  utraj0 = utraj_init;
%   options.time = xtraj_init.pp.breaks;
  options.time = linspace(xtraj_init.tspan(1),xtraj_init.tspan(2), N);
else
  
  %Try to come up with a reasonable trajectory
  x1 = [.3;1;pi/8-pi/16;pi/8;-pi/8;pi/8;zeros(6,1)];
  t1 = .5;
  N1 = floor(N/2);
  xvec = x0*ones(1,N1) + (x1-x0)*linspace(0,1,N1);
  N2 = N-N1;
  xvec = [xvec x1*ones(1,N2) + (xf-x1)*linspace(0,1,N2)];
  
  % xfmin = [.1;1;0;0;0;0;-inf(6,1)];
  
  tf0 = 2;
  utraj0 = PPTrajectory(foh(linspace(0,tf0,N),randn(3,N)));
  
  xvec = x0*ones(1,N1) + (x1-x0)*linspace(0,1,N1);
  xvec = [xvec, x1*ones(1,N2) + (xf-x1)*linspace(0,1,N2)];
  d = 10;
  traj0.x = PPTrajectory(foh(linspace(0,tf0,N),xvec));
  traj0.lambda = PPTrajectory(foh(linspace(0,tf0,N),[zeros(2,N); repmat([0;196.2;0;0]/100,1,N1-d) repmat([0;196.2;0;196.2]/2/100,1,2*d) repmat([0;0;0;196.2]/100,1,N2-d)]));
%   traj0.cLambda = PPTrajectory(foh(linspace(0,tf0,N),[repmat([0;196.2;0;0],1,N1-2) [zeros(1,4);randi(200,1,4);zeros(1,4);randi(200,1,4)] repmat([0;0;0;196.2],1,N2-2)]));
%   traj0.cLambda = PPTrajectory(foh(linspace(0,tf0,N),[repmat([0;196.2;0;0],1,N1-4) [zeros(1,8);randi(200,1,8);zeros(1,8);randi(200,1,8)] repmat([0;0;0;196.2],1,N2-4)]));
%   traj0.cLambda = PPTrajectory(foh(linspace(0,tf0,N),[repmat([0;196.2;0;0],1,N1) repmat([0;0;0;196.2],1,N2)]));
%   traj0.cLambda = PPTrajectory(foh(linspace(0,tf0,N),repmat([0;196.2;0;196.2]/2,1,N)));
% traj0.cLambda = PPTrajectory(foh(linspace(0,tf0,N),repmat([0;0;0;0],1,N)));

%   traj0.lambda = PPtrajectory(foh(linspace(0,tf0,N),zeros(2,N)));

  traj0.x = traj0.x.setOutputFrame(p.getStateFrame);
end
con.fixtime = 1;
con.u.lb = p.umin;
con.u.ub = p.umax;
con.x0.lb = [x0(1:5);-inf; -inf; 0; -inf(4,1)];
con.x0.ub = [x0(1:5);inf;  inf; 0; inf(4,1)];
con.xf.lb = xfmin;%[qf;-inf(6,1)];%[-inf(3,1); pi/2; -inf(4,1)];
con.xf.ub = xfmax;%[qf;inf(6,1)];%[inf(3,1); pi/2; inf(4,1)];
% con.x.ub = [inf(3,1);-pi/16;inf;-pi/16;inf(6,1)];
con.T.lb = 1;   
con.T.ub = 6;
con.noflight = 1;

if nargin > 3
  con.alphamult = alphamult;
else
  con.alphamult = 1;
end
 
if nargin > 4
  con.betamult = betamult;
else
  con.betamult = 1;
end

options.traj0 = traj0;

% options.method='implicit_dircol';
% options.method='implicit_dirtran2';
options.method='implicitdirtran';
options.MinorFeasibilityTolerance = 1e-6;
options.MajorFeasibilityTolerance = 1e-6;
options.MajorOptimalityTolerance = 5e-6;
if nargin > 5
  options.MajorIterationsLimit = maxIter;
else
  options.MajorIterationsLimit = 100;
end
options.IterationsLimit = 50000;
options.VerifyLevel=0;
options.SuperbasicsLimit=3000;
checkDependency('snopt');
snprint('snopt.out');
snsummary('summary.out')

if exist('traj0','var')
  options.xtape0 = 'tape';
else
  xtraj0 = 0;
end

% options.grad_method='numerical';

tic
%options.grad_test = true;
[utraj,xtraj,info] = p.trajectoryOptimization(@cost,@finalcost,x0,utraj0,con,options);

ltraj = xtraj.ltraj;
xtraj = xtraj.xtraj;
% if (info~=1) error('failed to find a trajectory'); end
toc

% t = xtraj.getBreaks();
% t = linspace(t(1),t(end),100);
% x = xtraj.eval(t);
% plot(x(1,:),x(2,:));

v = p.constructVisualizer;
if (nargout<1)
  playback(v,xtraj,struct('slider',true));
end

end



      function [g,dg] = cost(dt,x,u,sys);
        R = 1;
        g = dt*sum((R*u).*u,1);
        dg = [g/dt, zeros(1,size(x,1)),2*dt*u'*R];
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
        R = 5;
        h = R*t;
        dh = [R,zeros(1,size(x,1))];
        return;
      end
      
