function runFunnel

p = PendulumPlant();
p = setInputLimits(p,-inf,inf);

[utraj,xtraj]=runDircol(p);

Q=diag([10,1]);
[c,V] = tvlqr(p,xtraj,utraj,diag([10,1]),.1,diag([10,1]));

sys = feedback(p,c);

poly = taylorApprox(sys,xtraj,[],3);
%poly = taylorApprox(p,xtraj,utraj,3);

%ts = linspace(xtraj.tspan(1),xtraj.tspan(end)/8,21);
%plotVdot(poly,V,ts);

V=sampledFiniteTimeInvariance(poly,diag([10,1]),V,xtraj.getBreaks());

figure(1); clf; hold on;
plotFunnel(xtraj,V);
fnplt(xtraj); 

end

