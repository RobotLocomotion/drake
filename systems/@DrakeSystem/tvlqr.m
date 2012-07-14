function [ltvsys,V] = tvlqr(obj,xtraj,utraj,Q,R,Qf,options)

% implements the time-varying linear (or affine) quadratic regulator
%  note:  Qf can be an nxn matrix, a cell array (for initializing the
%  affine) or an PolynomialLyapunovFunction (e.g. as would happen if you hand it back
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

% create new coordinate frames
iframe = CoordinateFrame([obj.getStateFrame.name,' - x0(t)'],nX,obj.getStateFrame.prefix);
obj.getStateFrame.addTransform(AffineTransform(obj.getStateFrame,iframe,eye(nX),-xtraj));
iframe.addTransform(AffineTransform(iframe,obj.getStateFrame,eye(nX),xtraj));

oframe = CoordinateFrame([obj.getInputFrame.name,' + u0(t)'],nU,obj.getInputFrame.prefix);
oframe.addTransform(AffineTransform(oframe,obj.getInputFrame,eye(nU),utraj));
obj.getInputFrame.addTransform(AffineTransform(obj.getInputFrame,oframe,eye(nU),-utraj));


if (isa(Q,'double'))
  Q = ConstantTrajectory(Q);
end
if (isa(R,'double'))
  Ri = ConstantTrajectory(inv(R));
elseif isa(R,'Trajectory')
  Ri = inv(R);
else
  error('R must be a double or a trajectory object');
end
sizecheck(Ri,[nU,nU]);

typecheck(Q,'Trajectory');  
sizecheck(Q,[nX,nX]);

if iscell(Qf)
  % intentionally left blank
elseif isa(Qf,'double')
  sizecheck(Qf,[nX,nX]);
  Qf = {Qf,zeros(nX,1),0};
elseif isa(Qf,'PolynomialLyapunovFunction')
  Vf = extractQuadraticLyapunovFunction(Qf);
  Vf = Vf.inFrame(obj.getStateFrame);
  Vf = Vf.inFrame(iframe);

  Qf=cell(3,1);
  if isTI(Vf)
    Qf{1}=Vf.S;
    Qf{2}=Vf.s1;
    Qf{3}=Vf.s2;
  else
    Qf{1}=Vf.S.eval(tspan(end));
    Qf{2}=Vf.s1.eval(tspan(end));
    Qf{3}=Vf.s2.eval(tspan(end));
  end
else
  error('Qf must be a double, a 3x1 cell array, or a PolynomialLyapunovFunction');
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

Ssqrt = cellODE(@ode45,@(t,S)affineSdynamics(t,S,obj,Q,Ri,xtraj,utraj,xdottraj,options),tspan(end:-1:1),{Qf{1}^(1/2),Qf{2},Qf{3}});
Ssqrt = flipToPP(Ssqrt);
S = recompS(Ssqrt);
B = getBTrajectory(obj,Ssqrt{1}.getBreaks(),xtraj,utraj,options);

% note that this returns what we would normally call -K.  here u(t) = u_0(t) + K(t) (x(t) - x_0(t)) 
K = affineKsoln(S,Ri,B);

if (obj.getStateFrame ~= obj.getOutputFrame)  % todo: remove this or put it in a better place when I start doing more observer-based designs
  warning('designing full-state feedback controller but plant has different output frame than state frame'); 
end
  
ltvsys = AffineSystem([],[],[],[],[],[],[],K{1},K{2});
ltvsys = setInputFrame(ltvsys,iframe);
ltvsys = setOutputFrame(ltvsys,oframe);

if (nargout>1)
  V=QuadraticLyapunovFunction(ltvsys.getInputFrame,S{1},S{2},S{3});
end

end


function B = getBTrajectory(plant,ts,xtraj,utraj,options)
  nX = length(xtraj); nU = length(utraj);
  B = zeros([nX,nU,length(ts)]);
  for i=1:length(ts)
    x0=xtraj.eval(ts(i)); u0 = utraj.eval(ts(i));
    [f,df] = geval(@plant.dynamics,ts(i),x0,u0,options);
    B(:,:,i) = df(:,nX+1+(1:nU));
  end
  B = PPTrajectory(spline(ts,B));
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

function K = Ksoln(S,Ri,B)
  K = -Ri*B'*S;
end

function Sdot = affineSdynamics(t,S,plant,Qtraj,Ritraj,xtraj,utraj,xdottraj,options)
  x0 = xtraj.eval(t); u0 = utraj.eval(t); xdot0 = xdottraj.eval(t);
  Q=Qtraj.eval(t);  Ri=Ritraj.eval(t);
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
    
function S = recompS(Ssqrt)
  S{1} = Ssqrt{1}*Ssqrt{1}';
  S{2} = Ssqrt{2};
  S{3} = Ssqrt{3};
end

function K = affineKsoln(S,Ri,B)
  K{1} = -Ri*B'*S{1};
  K{2} = -.5*Ri*B'*S{2};
end

