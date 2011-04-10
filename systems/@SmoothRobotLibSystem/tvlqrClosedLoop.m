function [ltvsys,sys,xtraj,utraj,Vtraj,Vf]=tvlqrClosedLoop(obj,xtraj,utraj,Q,R,Vf)

[ltvsys,Vtraj] = tvlqr(obj,xtraj,utraj,Q,R,Vf);

sys = feedback(obj,ltvsys);

m = ltvsys.getNumStates();

if (m>0)
  xtraj = MixedTrajectory({PPTrajectory(zoh(xtraj.tspan,[0 0])),xtraj},{1:m,m+(1:obj.getNumStates())}); % add LTVcontroller state (t0)
end
utraj = FunctionHandleTrajectory(@(t)zeros(0),[0 0],xtraj.tspan);  % utraj is now empty

N = obj.getNumStates();
x = msspoly('x',m+N);
Vtraj = PolynomialTrajectory(@(t) subs(Vtraj.handle(t),x(1:N),x(m+(1:N))), Vtraj.getBreaks());

if (isnumeric(Vf))
  Vf = x(m+(1:N))'*Vf*x(m+(1:N));
else
  Vf = subs(Vf,x(1:N),x(m+(1:N)));
end
