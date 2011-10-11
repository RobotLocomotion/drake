function Vtraj = tvlyap(obj,xtraj,Q,Qf,tspan)
% implements the time-varying linear (or affine) quadratic regulator
%  note:  Qf can be an nxn matrix, a cell array (for initializing the
%  affine) or an msspoly (e.g. as would happen if you hand it back
%  Vtraj.eval(0) from a previous tvlqr trajectory).

typecheck(xtraj,'Trajectory');
sizecheck(xtraj,[getNumStates(obj),1]);

nX = xtraj.dim;
nU = getNumInputs(obj);
%tspan=xtraj.getBreaks(); %TODO: pass in options for tspan
%tspan = linspace(tspan(1), tspan(end), 100);

if (isa(Q,'double'))
  Q = PPTrajectory(zoh(tspan([1,end]),repmat(Q,[1 1 2])));
end

typecheck(Q,'Trajectory');  
sizecheck(Q,[nX,nX]);

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

odeoptions = odeset('RelTol',1e-6,'AbsTol',1e-12);
Ssqrt = cellODE(@ode45,@(t,S)affineSdynamics(t,S,obj,Q,xtraj,xdottraj),tspan(end:-1:1),{Qf{1}^(1/2),Qf{2},Qf{3}},odeoptions);
S = FunctionHandleTrajectory(@(t) recompS(Ssqrt.eval(t)),[nX,nX],tspan);
%S_nonAffine = matrixODE(@ode45,@(t,S)Sdynamics(t,S,obj,Q,xtraj),tspan(end:-1:1),Qf{1},odeoptions);

p_x=msspoly('x',nX);
p_t=msspoly('t',1);
%Sdot = @(t)Sdynamics(t,S.eval(t),obj,Q,xtraj);
%Vtraj = PolynomialTrajectory(@(t) lyapunov(t,S.eval(t),Sdot(t),xtraj.eval(t),p_x,p_t), tspan);
Sdot = @(t)recompSdot(Ssqrt.eval(t),affineSdynamics(t,Ssqrt.eval(t),obj,Q,xtraj,xdottraj));
Vtraj = PolynomialTrajectory(@(t) affineLyapunov(t,S.eval(t),Sdot(t),xtraj.eval(t),p_x,p_t), tspan);

end


function Sdot = Sdynamics(t,S,plant,Qtraj,xtraj)
  x0 = xtraj.eval(t);
  Q = Qtraj.eval(t); 
  nX = length(x0); nU = getNumInputs(plant);
  [f,df] = geval(@plant.dynamics,t,x0,zeros(nU,1));
  
  % f = xdot
  % xtraj.deriv(t) should equal f
  
  
  
  A = df(:,1+(1:nX));
  Sdot = -(Q + S*A + A'*S);
  if (min(eig(S))<0) error('S is not positive definite'); end
end

function Sdot = affineSdynamics(t,S,plant,Qtraj,xtraj,xdottraj)
  x0 = xtraj.eval(t); xdot0 = xdottraj.eval(t);
  Q=Qtraj.eval(t);
  nX = length(x0); nU = getNumInputs(plant);
  [xdot,df] = geval(@plant.dynamics,t,x0,zeros(nU,1));
  A = df(:,1+(1:nX));
  c = xdot - xdot0;
  Sdot{1} = -.5*Q*inv(S{1}')-A'*S{1};  % sqrt method
  Sorig = S{1}*S{1}';
  Sdot{2} = -(A'*S{2} + 2*Sorig*c);
  Sdot{3} = -S{2}'*c;  
  
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

function V=lyapunov(t,S,Sdot,x0,p_x,p_t)
  V = (p_x-x0)'*(S+Sdot*(p_t-t))*(p_x-x0);
end

function V=affineLyapunov(t,S,Sdot,x0,p_x,p_t)
  V = (p_x-x0)'*(S{1}+Sdot{1}*(p_t-t))*(p_x-x0) + (p_x-x0)'*(S{2}+Sdot{2}*(p_t-t)) + S{3}+Sdot{3}*(p_t-t);
end
