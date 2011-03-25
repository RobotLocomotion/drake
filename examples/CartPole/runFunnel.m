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

options = struct();
options.rho0_tau = 6;
options.max_iterations = 3;
V=sampledFiniteTimeInvariance(poly,.1*eye(4),V,xtraj.getBreaks(),options);

return;
figure(1); clf
Vsub = FunctionHandleTrajectory(@(t) subs(V.eval(t),poly.p_x([1 3]),[0;0]),[1 1],V.tspan);
plotFunnel(xtraj,Vsub);


end

