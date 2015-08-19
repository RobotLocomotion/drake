function tvlqrRestartTest

oldpath = addpath(fullfile(pwd,'..'));

p = PendulumPlant();
p = setInputLimits(p,-inf,inf);

[utraj,xtraj]=swingUpTrajectory(p);

Q=diag([10,1]);R=.1;
[c,V] = tvlqr(p,xtraj,utraj,Q,R,Q);

[breaks,coefs,l,k,d]=unmkpp(utraj.pp());
isplit = floor(l/2);
utraj1 = setOutputFrame(PPTrajectory(mkpp(breaks(1:isplit+1),coefs(1:isplit,:),d)),getInputFrame(p));
utraj2 = setOutputFrame(PPTrajectory(mkpp(breaks(isplit+1:end),coefs(isplit+1:end,:),d)),getInputFrame(p));

[breaks,coefs,l,k,d]=unmkpp(xtraj.pp()); coefs=reshape(coefs,prod(d),l,k);
xtraj1 = setOutputFrame(PPTrajectory(mkpp(breaks(1:isplit+1),coefs(:,1:isplit,:),d)),getStateFrame(p));
xtraj2 = setOutputFrame(PPTrajectory(mkpp(breaks(isplit+1:end),coefs(:,isplit+1:end,:),d)),getStateFrame(p));

[c2,V2] = tvlqr(p,xtraj2,utraj2,Q,R,Q);
[c1,V1] = tvlqr(p,xtraj1,utraj1,Q,R,V2);

if ~isequal(V.getPoly(0),V1.getPoly(0),1e-3)
  error('lyapunov candidates don''t match');
end

path(oldpath);
