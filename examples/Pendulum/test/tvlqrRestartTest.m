function tvlqrRestartTest

oldpath = addpath('..');

p = PendulumPlant();
p = setInputLimits(p,-inf,inf);

[utraj,xtraj]=swingUpTrajectory(p);

Q=diag([10,1]);R=.1;
[c,V] = tvlqr(p,xtraj,utraj,Q,R,Q);

[breaks,coefs,l,k,d]=unmkpp(utraj.pp());
isplit = floor(l/2);
utraj1 = PPTrajectory(mkpp(breaks(1:isplit+1),coefs(1:isplit,:),d));
utraj2 = PPTrajectory(mkpp(breaks(isplit+1:end),coefs(isplit+1:end,:),d));

[breaks,coefs,l,k,d]=unmkpp(xtraj.pp()); coefs=reshape(coefs,prod(d),l,k);
xtraj1 = PPTrajectory(mkpp(breaks(1:isplit+1),coefs(:,1:isplit,:),d));
xtraj2 = PPTrajectory(mkpp(breaks(isplit+1:end),coefs(:,isplit+1:end,:),d));

[c2,V2] = tvlqr(p,xtraj2,utraj2,Q,R,Q);
[c1,V1] = tvlqr(p,xtraj1,utraj1,Q,R,V2);

if ~isequal(V.getPoly(0),V1.getPoly(0),1e-3)
  error('lyapunov candidates don''t match');
end

path(oldpath);
