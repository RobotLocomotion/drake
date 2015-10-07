function [xtraj,utraj,ltraj,v,p]=runVarTimeDirtran(xtraj_init, utraj_init, ltraj_init)

% p = AcrobotPlant();
p = PlanarRigidBodyManipulator('spring_flamingo.urdf');
N = 20;
% N = 100;
q0 = [0; .84; 0; 0;0;0;0;0;0];
% q0 = [0;cos(pi/16);pi/16;-pi/8;0*pi/8;-pi/8];
% qf = [sin(pi/8); cos(pi/8); -pi/8; 0; pi/4; 0];

%jerry.mat order
% x
% z
% pitch
% dx/dt
% dz/dt
% dpitch/dt
% hip, dhip/dt
% knee, dknee/dt
% ankle, dankle/dt
% repeat leg

% state order:
% x
% z
% body pitch
% hip
% knee1
% ankle
% hip
% knee
% ankle

con.linperiodic = zeros(18);
con.linperiodic(2,2) = 1; %z 
con.linperiodic(3,3) = 1; %pitch
con.linperiodic(4:6,7:9) = eye(3); %leg joints w/symmetry
con.linperiodic(7:9,4:6) = eye(3); %leg joints w/symmetry

con.linperiodic(10:12,10:12) = eye(3); %x,z,pitch velocities
con.linperiodic(13:15,16:18) = eye(3); %leg joints w/symmetry
con.linperiodic(16:18,13:15) = eye(3); %leg joints w/symmetry

%CHANGED Q0, XFMAX, XFMIN, X.UB

%read to blue
x0 = [q0;zeros(9,1)];
xf = x0 + [.6;zeros(17,1)];

xfmin = [-inf(18,1)];
xfmax = [-.57;inf(17,1)];  %walking negative x direction

if exist('xtraj_init')
  xtraj0.x = xtraj_init;
  xtraj0.lambda = ltraj_init;
  utraj0 = utraj_init;
%   options.time = xtraj_init.pp.breaks;
  options.time = linspace(xtraj_init.tspan(1),xtraj_init.tspan(2), N);

  %randomly resample time interview, preserving initial knot points
  while length(options.time) < N
    ti = options.time(1) + (options.time(end) - options.time(1))*rand;
    options.time = sort([options.time ti]);
  end
else
  t0 = .65;
  tf = 1.65;  
  load jerry.mat
  t = linspace(0,tf-t0,N);
  x = traj.eval(t+t0);
  x = x([1 2 3 7 9 11 13 15 17 4 5 6 8 10 12 14 16 18],:);
  x(1,:) = x(1,:) - x(1,1);
  x(4:9,:) = -x(4:9,:);
  x(13:18,:) = -x(13:18,:);
  xtraj0.x = PPTrajectory(foh(t,x));
  
  
  
  tf0 = tf-t0;
  load full_input;
%  utraj0 = PPTrajectory(foh(t,interp1(t_input,u_input,t+t0)'));
  utraj0 = PPTrajectory(foh(linspace(0,tf0,N),randn(6,N)));

  N1 = floor(N/2);
  N2 = N-N1;
%   %Try to come up with a reasonable trajectory
%   x1 = [-.2;.84;pi/16;-pi/4;pi/8;zeros(13,1)]
%   tf0 = 2;
%   xvec = x0*ones(1,N1) + (x1-x0)*linspace(0,1,N1);
%   xvec = [xvec, x1*ones(1,N2) + (xf-x1)*linspace(0,1,N2)];
%   xtraj0.x = PPTrajectory(foh(linspace(0,tf0,N),xvec));
  d = 7;
  xtraj0.lambda = PPTrajectory(foh(linspace(0,tf0,N),[repmat([0;0;0;0;0;7;0;7]/100,1,N1-d) zeros(8,2*d) repmat([0;7;0;7;0;0;0;0]/100,1,N2-d)]));
end
% con.noflight = 1;
con.fixtime = 2;
con.u.lb = p.umin;
con.u.ub = p.umax;
con.x0.lb = [x0(1); x0(2); -.1; -inf(4,1); -0; -inf; -5; -.1; -5*ones(7,1)];
con.x0.ub = [x0(1); x0(2);  .1;  inf(4,1);  0;  inf;  5;  .1;  5*ones(7,1)];
con.xf.lb = xfmin;
con.xf.ub = xfmax;
con.x.ub = [inf; .88; pi/6; inf; pi/3; pi/2;  inf; pi/3; pi/2; inf(9,1)];
con.x.lb = -[inf; inf; pi/6; inf; 0; pi/2;  inf; 0; pi/2; inf(9,1)];
con.T.lb = .8;   
con.T.ub = 1.5;

con.alphamult = 0;
con.betamult = 0;

% options.method='implicit_dircol';
% options.method='implicit_dirtran2';
options.method='vartime_implicit_dirtran';
options.MinorFeasibilityTolerance = 1e-6;
options.MajorFeasibilityTolerance = 1e-6;
options.MajorOptimalityTolerance = 1e-6;
options.MajorIterationsLimit = 50;
options.IterationsLimit =50000;
options.VerifyLevel=0;
options.SuperbasicsLimit=3000;
snprint('snopt.out');

if exist('xtraj0','var')
  options.xtape0 = 'tape';
else
  xtraj0 = 0;
end

tic
[utraj,xtraj,info,ltraj] = p.trajectoryOptimization(@cost,@finalcost,x0,utraj0,con,options,xtraj0);
toc

v = p.constructVisualizer;

end



      function [g,dg] = cost(dt,x,u,sys);
        % Use cost of transport
        R = 1;
%         g = 0;
%         dg = zeros(1,25);
%         for i=1:6,
%           gi = dt*u(i)*x(12+i);
%           if gi >= 0
%             g = g + gi;
%             dg(13+i) = dt*u(i);
%             dg(19+i) = dt*x(12+i);
%           end
%         end
%         dg(1) = g/dt;
        g = dt*sum(abs((R*u).*x(13:18)),1);
        dg = [g/dt, zeros(1,12), dt*abs(u)'.*sign(x(13:18))', dt*sign(u)'.*abs(x(13:18))'];
        
%         R = 1;
%         g = dt*sum((R*u).*u,1);
%         dg = [g/dt, zeros(1,size(x,1)),2*dt*u'*R];
      end
      
      function [h,dh] = finalcost(t,x)
        R = 20;
        h = R*t;
        dh = [R,zeros(1,size(x,1))];
        return;
      end
      
