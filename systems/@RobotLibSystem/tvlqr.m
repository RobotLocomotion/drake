function ltvsys = tvlqr(obj,xtraj,utraj,Q,R,Qf)

typecheck(xtraj,'Trajectory');
sizecheck(xtraj,[getNumStates(obj),1]);
typecheck(utraj,'Trajectory');
sizecheck(utraj,[getNumInputs(obj),1]);

nX = xtraj.dim;
nU = utraj.dim;
tspan=utraj.getBreaks();

if (isa(Q,'double'))
  Q = PPTrajectory(zoh(tspan([1,end]),repmat(Q,[1 1 2])));
end
if (isa(R,'double'))
  R = PPTrajectory(zoh(tspan([1,end]),repmat(R,[1 1 2])));
end

typecheck(Q,'Trajectory');  
sizecheck(Q,[nX,nX]);
typecheck(R,'Trajectory');
sizecheck(R,[nU,nU]);
typecheck(Qf,'double');
sizecheck(Qf,[nX,nX]);

if (~isCT(obj)) error('only handle CT case so far'); end

S = matrixODE(@ode45,@(t,S)Sdynamics(t,S,obj,Q,R,xtraj,utraj),tspan(end:-1:1),Qf);
K = FunctionHandleTrajectory(@(t)Ksoln(t,obj,S,R,xtraj,utraj),[nX nU],tspan);
Sdot = FunctionHandleTrajectory(@(t)Sdynamics(t,S.eval(t),obj,Q,R,xtraj,utraj),[nX nX],tspan);

ltvsys = LTVControl(xtraj,utraj,K,S,Sdot);

end

function Sdot = Sdynamics(t,S,plant,Qtraj,Rtraj,xtraj,utraj)
x0 = xtraj.eval(t); u0 = utraj.eval(t);
Q = Qtraj.eval(t); Ri = inv(Rtraj.eval(t));
nX = length(x0); nU = length(u0);
df = plant.dynamicsGradients(t,x0,u0);
A = df{1}(:,1+(1:nX));
B = df{1}(:,nX+1+(1:nU));
Sdot = -(Q - S*B*Ri*B'*S + S*A + A'*S);
end

function K = Ksoln(t,plant,Straj,Rtraj,xtraj,utraj)
S = Straj.eval(t); Ri = inv(Rtraj.eval(t)); x0=xtraj.eval(t); u0 = utraj.eval(t);
nX = length(x0); nU = length(u0);
df = plant.dynamicsGradients(t,x0,u0);
B = df{1}(:,nX+1+(1:nU));
K = Ri*B'*S;
end
    