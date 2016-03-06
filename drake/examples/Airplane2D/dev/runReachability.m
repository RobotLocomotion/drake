function Vsall = runReachability(p,doTest)

if (nargin<1)
  p = PlanePlant();
  doTest = 1; % Perform test to make sure trajectories stay inside funnel
end

if (nargin < 2)
    doTest = 1;
end

% Trajectory library dircol
x0 = [0;0;0;0];
tf0 = 0.3;
xf = [-0.3; 2; 0; 0];
    
% generate a random trajectory
utraj0 = PPTrajectory(foh(linspace(0,tf0,11),2*randn(1,11)));

%con.u.lb = p.umin;
%con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
con.T.lb = 0.1;   
con.T.ub = 1.3;

options.method='dircol';
tic
%options.grad_test = true;
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

ts = xtraj.getBreaks();

    
% Plot trajectories
figure(1)
hold on
grid on
fnplt(xtraj,[1 2]);

% TVLQR
Q = diag([1 1 10 10]);
R=0.1;
Q0 = diag([1 1 1 1]);

tv = tvlqr(p,xtraj,utraj,Q,R,Q0);

% Generate cubic approx. of closed loop dynamics
sysCl = feedback(p,tv);
poly = taylorApprox(sysCl,xtraj,[],3);

% Do verification
options.degV = 2;
options.degW = 4;
options.degq1 = options.degV;
options.degq01 = options.degW - 2;
options.degqT = options.degV - 2;
options.degs0 = options.degW - 2;
options.backoff_percent = 5;

x = poly.getOutputFrame.getPoly;
% Initial condition set (This is the initial condition set defined by some
% sub-level set of the tvlqr S at time 0 - I chose this so I could compare
% with our usual sos method. Obviously, with this approach, we can specify
% any initial condition set we want).
gT = 1 - (1.1961*x(1)^2+1.203*x(2)^2-0.002394*x(2)*x(1)+7.2875*x(3)^2-4.3514*x(3)*x(1)-0.64564*x(3)*x(2)+1.0289*x(4)^2-0.24223*x(4)*x(1)-0.041337*x(4)*x(2)+0.8859*x(4)*x(3));
R = 3; 


disp('Starting verification')
[Wopt,xs] = sampledFiniteTimeReachability(poly,ts,xtraj,gT,R,options);

% Plot funnel
disp('Plotting funnel')
figure(1)
xlabel('x')
ylabel('y')
hold on
for k = 1:length(Wopt)
    if isempty(Wopt{k})
        continue
    end
    x0 = xtraj.eval(ts(k));
    wstr = sdpvar2str(clean(replace(Wopt{k},xs,[xs(1)-x0(1);xs(2)-x0(2);0;0]),1e-6));
    wstr = strcat(wstr{1},'-1');
    wstr = regexprep(wstr,'x\(1\)','x1');
    wstr = regexprep(wstr,'x\(2\)','x2');
    ezplot(wstr);
    drawnow;
end
title('Funnel')

Vsall = [];
if doTest
    % Make sure trajectories stay inside funnel
    disp('Testing to make sure funnel is not violated...')
    % Check that funnel is not violated 
    % Note that the one super-level set defines the funnel, not sub-level set :)
    figure(2)
    xlabel('t')
    ylabel('W(t)')
    title('W vs t')
    grid on
    axis([0,ts(end),0,2])
    xinit=getLevelSet(x,-gT+1,[0;0;0;0]);
    for k = 1:10 %length(xinit)
        trajSim = sysCl.simulate(ts,0.99*xinit(:,k));
        Vs = [];
        for i = 1:length(ts)
            Vs = [Vs,double(replace(Wopt{i},xs,trajSim.eval(ts(i)) - xtraj.eval(ts(i))))];
        end
        hold on
        plot(ts,Vs);
        Vsall = [Vsall,Vs];
    end

    plot(ts,ones(size(ts)),'ro');
end

end

function [g,dg] = cost(t,x,u)
        R = 0;
        g = u'*R*u;
        %g = sum((R*u).*u,1);
        %dg = [zeros(1,1+size(x,1)),2*u'*R];
        dg = zeros(1, 1 + size(x,1) + size(u,1));
end
      
function [h,dh] = finalcost(t,x)
    h = t;
    dh = [1,zeros(1,size(x,1))];
end






