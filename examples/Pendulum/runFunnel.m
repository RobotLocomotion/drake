function runFunnel

p = PendulumPlant();
p = setInputLimits(p,-inf,inf);

[utraj,xtraj]=runDircol(p);

[c,V] = tvlqr(p,xtraj,utraj,diag([10,1]),.1,diag([10,1]));

sys = feedback(p,c);

poly = taylorApprox(sys,xtraj,[],3);
%poly = taylorApprox(p,xtraj,utraj,3);

%ts = linspace(xtraj.tspan(1),xtraj.tspan(end)/8,21);
%plotVdot(poly,V,ts);

V=sampledFiniteTimeInvariance(poly,.1*eye(2),V,xtraj.getBreaks());

return
figure(1); clf
plotFunnel(xtraj,V);


end

