function V = tvlyap(obj,xtraj,Q,Qf) % note: no options yet

typecheck(xtraj,'Trajectory');
sizecheck(xtraj,[getNumStates(obj),1]);

nX = xtraj.dim;
nU = getNumInputs(obj);
tspan=xtraj.getBreaks(); %TODO: pass in options for tspan
%tspan = linspace(tspan(1), tspan(end), 100);

% create new coordinate frame
fr = CoordinateFrame([obj.getStateFrame.name,' - x0(t)'],nX,obj.getStateFrame.prefix);
obj.getStateFrame.addTransform(AffineTransform(obj.getStateFrame,fr,eye(nX),-xtraj));
fr.addTransform(AffineTransform(fr,obj.getStateFrame,eye(nX),xtraj));

if (isa(Q,'double'))
  Q = ConstantTrajectory(Q);
end

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
  Vf = Vf.inFrame(fr);

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

odeoptions = odeset('RelTol',1e-6,'AbsTol',1e-12);
Ssqrt = cellODE(@ode45,@(t,S)affineSdynamics(t,S,obj,Q,xtraj,xdottraj),tspan(end:-1:1),{Qf{1}^(1/2),Qf{2},Qf{3}},odeoptions);
%S_nonAffine = matrixODE(@ode45,@(t,S)Sdynamics(t,S,obj,Q,xtraj),tspan(end:-1:1),Qf{1},odeoptions);
Ssqrt = flipToPP(Ssqrt);
S = recompS(Ssqrt);

V=QuadraticLyapunovFunction(ltvsys.getInputFrame,S{1},S{2},S{3});

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


