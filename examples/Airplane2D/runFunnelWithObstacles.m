function [V, utraj,xtraj]=runFunnelWithObstacles(p)

if (nargin<1)
  p = PlanePlant();
end

p = setInputLimits(p,-inf,inf);

[utraj, xtraj, field] = runDircolWithObs(p);
% load planeTraj.mat %utraj xtraj field
hold on;
h=fnplt(xtraj,[1 2]);
set(h,'Color',[1 0 0]);
hold off
delete(h);

Q = (1/10)*diag([10 10 1 1]); %diag([1 1 10 .1]);
R=.1; % 0.01
Qf = Q; %diag([1 1 1 10]); %diag([1 1 10 .1]);

options = struct();
options.sqrtmethod = true;
options.x0 = xtraj;
%options.max_iterations = 3;

% options.x0 = xtraj;
% Vxframe = V.inFrame(p.getStateFrame);



% [~,~]=tvlqr(p,xtraj,utraj,Q,R,Qf,options);
[c,V]=tvlqr(p,xtraj,utraj,Q,R,Qf,options);
options.x0 = xtraj;
Vxframe = V.inFrame(p.getStateFrame);

%plotFunnel(Vxframe,options);

%keyboard;
%plotFunnel(Vxframe,options);

poly = taylorApprox(feedback(p,c),xtraj,[],3);

options.stability = true;
options.max_iterations = 5;
options.converged_tol = -1; % Run for all iterations
options.rho0_tau = 13; % 5
ts = V.S.getBreaks;
ts = linspace(ts(1),ts(end),11);
%keyboard;
V=sampledFiniteTimeVerification(poly,ts,Qf,V,options);
disp('done');

figure(25);
options.plotdims=[1 2];
options.inclusion='slice';
Vxframe = V.inFrame(p.getStateFrame());
plotFunnel(Vxframe,options);
h=fnplt(xtraj,[1 2]);
set(h,'Color',[1 0 0]);

% return;
% keyboard;

% todo: implement trimming properly
ts = V.S.getBreaks();
rho(length(ts))=1;

vert=[];
for i=1:field.number_of_obstacles
  obstacle = field.obstacles{i};
  vert=[vert,[obstacle.xvector;obstacle.yvector]];
end

% iterate over t
for i=fliplr(1:length(ts)-1)

  rho(i) = rho(i+1);

  % iterate over every polygon
%  for j=1:field.number_of_obstacles
%    obstacle = field.obstacles{j};

%    for k=1:length(obstacle.xvector)
%       kn = mod(k,length(obstacle.xvector))+1;
%       x1 = [obstacle.xvector(k);obstacle.yvector(k)];
%       x2 = [obstacle.xvector(kn); obstacle.yvector(kn)];
%
%       %  For each convex polygon I need a matrix A and vector b such that:
%       % Ax <= b ==> you're in the polygon.
%       A = [x2(2)-x1(2); -(x2(1)-x1(1))]';
%       b = A*x1;
%       % pad with zeros
%       A = -[A,zeros(1,2)];
%
%       Vt = V.eval(ts(i));
%       Vmin = blah(Vt,V.p_x,A,b)
      x0 = xtraj.eval(ts(i));
      x = [vert;repmat(x0(3:4),1,size(vert,2))];
      Vvert = [];
      for k = 1:length(vert)
        Vvert = [Vvert,Vxframe.eval(ts(i),x(:,k))];
      end
      if (min(Vvert)<rho(i))
        rho(i) = min(Vvert);
      end
%    end
%  end
end

% todo: still need to update this
% Vtrim = PolynomialTrajectory(@(t) V.eval(t)/ppvalSafe(foh(ts,rho),t),ts);
Vtrim = Vxframe*PPTrajectory(foh(ts,1./rho));

% keyboard;
clf;
v = PlaneVisualizer(p,field);
v.draw(0,x0);
plotFunnel(Vtrim,options);
h=fnplt(xtraj,[1 2]);
set(h,'Color',[1 0 0]);

end

function [g,dg] = cost(t,x,u)
    R = 1;
    g = u'*R*u;
    %g = sum((R*u).*u,1);
    dg = [zeros(1,1+size(x,1)),2*u'*R];
    %dg = zeros(1, 1 + size(x,1) + size(u,1));
end

function [h,dh] = finalcost(t,x)
    h = t;
    dh = [1,zeros(1,size(x,1))];
end


function [Vmin,xopt] = blah(V,x,A,b)
  n = length(x);
  % V = 0.5 x'Hx + f'x + b
  % Compute x0
  H = double(diff(diff(V,x)',x));
  f = double(subs(diff(V,x),x,0*x));

  opt = optimset('LargeScale','off','Display','off');
  xopt = quadprog(H,f,[-A zeros(size(A,1),n-size(A,2))],-b,[],[],[],[],[],opt);
  Vmin = double(subs(V,x,xopt));
end

% TIMEOUT 1500
% NOTEST % even that timeout was not sufficient.  quieting this test to speed up the build server.
