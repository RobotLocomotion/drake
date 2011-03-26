function runFunnel

p = CartPolePlant();
p = setInputLimits(p,-inf,inf);

[utraj,xtraj]=runDircol(p);

[c,V] = tvlqr(p,xtraj,utraj,diag([10,10,1,1]),.1,diag([10,10,1,1]));

sys = feedback(p,c);

poly = taylorApprox(sys,xtraj,[],3);
%poly = taylorApprox(p,xtraj,utraj,3);

%ts = linspace(xtraj.tspan(1),xtraj.tspan(end)/8,21);
%plotVdot(poly,V,ts);

close all;
options = struct();
options.rho0_tau = 10;
options.max_iterations = 3;
V=sampledFiniteTimeInvariance(poly,.1*eye(4),V,xtraj.getBreaks(),options);

figure(1); clf
plotFunnel(xtraj,V,[2 4]);
fnplt(xtraj,[2 4]); 
xlabel('theta');
ylabel('theta dot');
title('CartPole Swing-up Funnel')


end

