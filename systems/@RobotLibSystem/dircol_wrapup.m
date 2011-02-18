function [utraj,xtraj] = dircol_wrapup(sys,w,t,nX,nU)

t = t*w(1);
nT = length(t);

x = reshape(w(1 + (1:nT*nX)),nX,nT);
u = reshape(w((1 + nT*nX + 1):end),nU,nT);

utraj = PPTrajectory(foh(t,u));
for i=1:nT
  xdot(:,i) = sys.dynamics(t(i),x(:,i),u(:,i));
end
xtraj = PPTrajectory(pchipDeriv(t,x,xdot));

end