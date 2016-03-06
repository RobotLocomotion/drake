function runFunnel

oldpath = addpath(fullfile(pwd,'..'));

p = GliderPlant();
p = setInputLimits(p,-inf,inf);

[utraj,xtraj]=runDircol(p);

%  sqrt method seems to not work for this dynamics.
try 
Q = diag([10 10 1 0 1 1 1]);
[c,V] = tvlqr(p,xtraj,utraj,Q,.1,Q);

sys = feedback(p,c);

x=simulate(sys,xtraj.tspan);

v=GliderVisualizer();
v.playback(x);
catch exception
  path(oldpath);
  rethrow(exception);
end

path(oldpath);
return;

poly = taylorApprox(sys,xtraj,[],3);
%poly = taylorApprox(p,xtraj,utraj,3);

%ts = linspace(xtraj.tspan(1),xtraj.tspan(end)/8,21);
%plotVdot(poly,V,ts);

V=sampledFiniteTimeVerification(poly,diag([10,1]),V,xtraj.getBreaks());

figure(1); clf; hold on;
plotFunnel(V);
fnplt(xtraj); 

end

