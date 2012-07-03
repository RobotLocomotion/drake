function [V, utraj,xtraj]=runFunnel(p)

if (nargin<1)
  p = PlanePlant();
end

p = setInputLimits(p,-inf,inf);

% OKTOFAIL
[utraj, xtraj] = runDircol(p);

Q = eye(4);
R=1;
%Qf = eye(4);

options = struct();
options.rho0_tau = 10;
%options.max_iterations = 3;


[c,V]=tvlqr(p,xtraj,utraj,Q,R,diag([1 1 10 10]));
sys = feedback(p,c);
utraj = ConstantTrajectory(zeros(p.getNumInputs,1)); utraj=utraj.setOutputFrame(p.getInputFrame); 
poly = taylorApprox(sys,xtraj,utraj,3);

%options.stability = true;

V=sampledFiniteTimeVerification(poly,Vf,V,xtraj.getBreaks(),xtrajClosedLoop,utraj,options);
disp('done');
V


plotFunnel(V,xtraj,[1 2]);
fnplt(xtraj,[1 2]); 

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


