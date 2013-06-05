function [V, utraj,xtraj]=runFunnelWithObstacles(p)

if (nargin<1)
  p = PlanePlant();
end

p = setInputLimits(p,-inf,inf);

[utraj, xtraj, field] = runDircolWithObs(p);
hold on;
h=fnplt(xtraj,[1 2]);
set(h,'Color',[1 0 0]);
hold off
delete(h);

Q = diag([1 1 10 .1]); %diag([10 10 1 .1]);
R=.01;
Qf = diag([1 1 10 .1]); %diag([1 1 10 10]);

options = struct();
options.rho0_tau = 10;
%options.max_iterations = 3;

[c,V]=tvlqr(p,xtraj,utraj,Q,R,Q);
poly = taylorApprox(feedback(p,c),xtraj,[],3);

options.stability = true;
options.max_iterations=20;
options.converged_tol =1e-5;
V=sampledFiniteTimeVerification(poly,xtraj.getBreaks(),diag([1 1 10 10]),V,options);
disp('done');

figure(25);
options.plotdims=[1 2];
options.inclusion='projection';
plotFunnel(V.inFrame(p.getStateFrame()),options);
h=fnplt(xtraj,[1 2]); 
set(h,'Color',[1 0 0]);

return;  

% todo: implement trimming properly

ts = V.getBreaks();
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
      Vvert = V.eval(ts(i),x);
      if (min(Vvert)<rho(i))
        rho(i) = min(Vvert);
      end
%    end
%  end
end

% todo: still need to update this
Vtrim = PolynomialTrajectory(@(t) V.eval(t)/ppvalSafe(foh(ts,rho),t),ts);

clf;
v = PlaneVisualizer(p,field);
v.draw(0,x0);
plotFunnel(Vtrim.inFrame(p.getStateFrame()));
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
