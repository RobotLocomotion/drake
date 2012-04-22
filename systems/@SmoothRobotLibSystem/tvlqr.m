function [ltvsys,Vtraj] = tvlqr(obj,xtraj,utraj,Q,R,Qf,options)

% implements the time-varying linear (or affine) quadratic regulator
%  note:  Qf can be an nxn matrix, a cell array (for initializing the
%  affine) or an msspoly (e.g. as would happen if you hand it back
%  Vtraj.eval(0) from a previous tvlqr trajectory).

if (nargin<7) options=struct(); end
% no real options here, but options get's passed through to geval

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

switch class(Qf)
  case 'cell'
  case 'double'
    sizecheck(Qf,[nX,nX]);
    Qf = {Qf,zeros(nX,1),0};
  case 'msspoly'
    Vf=Qf;
    sizecheck(Vf,1);
    x=decomp(Vf);
    Vf=subss(Vf,x,x+xtraj.eval(tspan(end)));
    Qf=cell(3,1);
    Qf{1}=double(.5*subs(diff(diff(Vf,x)',x),x,0*x));
    Qf{2}=double(subs(diff(Vf,x),x,0*x))';
    Qf{3}=double(subs(Vf,x,0*x));
  otherwise
    error('Qf must be a double, a 3x1 cell array, or an msspoly');
end
sizecheck(Qf,3);
typecheck(Qf{1},'double');
sizecheck(Qf{1},[nX,nX]);
typecheck(Qf{2},'double');
sizecheck(Qf{2},[nX,1]);
typecheck(Qf{3},'double');
sizecheck(Qf{3},1);
  
if (~isCT(obj)) error('only handle CT case so far'); end

xdottraj = fnder(xtraj);

%Sold = matrixODE(@ode45,@(t,S)Sdynamics(t,S,obj,Q,R,xtraj,utraj,options),tspan(end:-1:1),Qf);
%K = FunctionHandleTrajectory(@(t)Ksoln(t,obj,S,R,xtraj,utraj,options),[nX nU],tspan);
Ssqrt = cellODE(@ode45,@(t,S)affineSdynamics(t,S,obj,Q,R,xtraj,utraj,xdottraj,options),tspan(end:-1:1),{Qf{1}^(1/2),Qf{2},Qf{3}});
S = FunctionHandleTrajectory(@(t) recompS(Ssqrt.eval(t)),[nX,nX],tspan);
K = FunctionHandleTrajectory(@(t)affineKsoln(t,obj,S,R,xtraj,utraj,options),[nX nU],tspan);

ltvsys = LTVControl(xtraj,utraj,K); %,S,Sdot);

if (nargout>1)
  p_x=msspoly('x',nX);
  p_t=msspoly('t',1);
%  Sdotold = @(t)Sdynamics(t,Sold.eval(t),obj,Q,R,xtraj,utraj);
%  Vtraj = FunctionHandleTrajectory(@(t) p_x'*(S.eval(t) + Sdot(t)*(p_t-t))*p_x, [1 1],tspan);
  Sdotsqrt = @(t)affineSdynamics(t,Ssqrt.eval(t),obj,Q,R,xtraj,utraj,xdottraj,options);
  Sdot = @(t) recompSdot(Ssqrt.eval(t),Sdotsqrt(t));
  Vtraj = PolynomialTrajectory(@(t) affineLyapunov(t,S.eval(t),Sdot(t),xtraj.eval(t),p_x,p_t), tspan);
  % todo: i could dig into the ODESolution with taylorvar to get higher
  % order, if Vddot was ever needed.
end

end



function Sdot = Sdynamics(t,S,plant,Qtraj,Rtraj,xtraj,utraj,options)
  x0 = xtraj.eval(t); u0 = utraj.eval(t);
  Q = Qtraj.eval(t); Ri = inv(Rtraj.eval(t));
  nX = length(x0); nU = length(u0);
  [f,df] = geval(@plant.dynamics,t,x0,u0,options);
  A = df(:,1+(1:nX));
  B = df(:,nX+1+(1:nU));
  Sdot = -(Q - S*B*Ri*B'*S + S*A + A'*S);
  if (min(eig(S))<0) warning('S is not positive definite'); end
end

function K = Ksoln(t,plant,Straj,Rtraj,xtraj,utraj,options)
  S = Straj.eval(t); Ri = inv(Rtraj.eval(t)); x0=xtraj.eval(t); u0 = utraj.eval(t);
  nX = length(x0); nU = length(u0);
  [f,df] = geval(@plant.dynamics,t,x0,u0,options);
  B = df(:,nX+1+(1:nU));
  K = Ri*B'*S;
end

function Sdot = affineSdynamics(t,S,plant,Qtraj,Rtraj,xtraj,utraj,xdottraj,options)
  x0 = xtraj.eval(t); u0 = utraj.eval(t); xdot0 = xdottraj.eval(t);
  Q=Qtraj.eval(t);  Ri=inv(Rtraj.eval(t));
  nX = length(x0); nU = length(u0);
  [xdot,df] = geval(@plant.dynamics,t,x0,u0,options);
  A = df(:,1+(1:nX));
  B = df(:,nX+1+(1:nU));
  c = xdot - xdot0;
%  Sdot{1} = -(Q - S{1}*B*Ri*B'*S{1} + S{1}*A + A'*S{1});
%  if (min(eig(S{1}))<0) warning('S is not positive definite'); end
  Sdot{1} = -.5*Q*inv(S{1}')-(A' - .5*S{1}*S{1}'*B*Ri*B')*S{1};  % sqrt method
  Sorig = S{1}*S{1}';
  Sdot{2} = -((A'-Sorig*B*Ri*B')*S{2} + 2*Sorig*c);
  Sdot{3} = -(-S{2}'*B*Ri*B'*S{2}/4 + S{2}'*c);  
end
    
function K = affineKsoln(t,plant,Straj,Rtraj,xtraj,utraj,options)
  S = Straj.eval(t); Ri = inv(Rtraj.eval(t)); x0=xtraj.eval(t); u0 = utraj.eval(t);
  nX = length(x0); nU = length(u0);
  [f,df] = geval(@plant.dynamics,t,x0,u0,options);
  B = df(:,nX+1+(1:nU));
  K{1} = Ri*B'*S{1};
  K{2} = .5*Ri*B'*S{2};
end

function S = recompS(Ssqrt)
  S{1} = Ssqrt{1}*Ssqrt{1}';
  S{2} = Ssqrt{2};
  S{3} = Ssqrt{3};
end

function Sdot = recompSdot(Ssqrt,Sdotsqrt)
  Sdot{1} = Sdotsqrt{1}*Ssqrt{1}' + Ssqrt{1}*Sdotsqrt{1}';
  Sdot{2} = Sdotsqrt{2};
  Sdot{3} = Sdotsqrt{3};
end

function V=affineLyapunov(t,S,Sdot,x0,p_x,p_t)
  V = (p_x-x0)'*(S{1}+Sdot{1}*(p_t-t))*(p_x-x0) + (p_x-x0)'*(S{2}+Sdot{2}*(p_t-t)) + S{3}+Sdot{3}*(p_t-t);
end

