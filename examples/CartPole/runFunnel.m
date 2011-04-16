function runFunnel

p = CartPolePlant();
p = setInputLimits(p,-inf,inf);

[utraj,xtraj]=runDircol(p);

Q=diag([10,10,1,1]); R=.1;

close all;
options = struct();
options.rho0_tau = 10;
options.max_iterations = 3;

% for debugging (the old way)
%[ltvsys,Vtraj] = tvlqr(p,xtraj,utraj,Q,R,Q);
%ltvsys = LTVControlTest(ltvsys.x0,ltvsys.u0,ltvsys.K);
%psystest = taylorApprox(feedback(p,ltvsys),xtraj,[],3);
%try 
%V=sampledFiniteTimeInvariance(psystest,Q,Vtraj,xtraj.getBreaks());
%catch
%  disp('funnel code threw error... continuing');
%end
% end debugging

[tv,sys,xtraj,utraj,V,Vf] = tvlqrClosedLoop(p,xtraj,utraj,Q,R,Q);
poly = taylorApprox(sys,xtraj,[],3);

V=sampledFiniteTimeVerification(poly,Vf,V,xtraj.getBreaks(),xtraj,utraj,options);

figure(1); clf
plotFunnel(V,xtraj,[2 4]);
fnplt(xtraj,[2 4]); 
xlabel('theta');
ylabel('theta dot');
title('CartPole Swing-up Funnel')


end

