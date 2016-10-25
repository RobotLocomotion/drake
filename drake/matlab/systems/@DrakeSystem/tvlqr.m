function [ltvsys,V] = tvlqr(obj,xtraj,utraj,Q,R,Qf,options)

% implements the time-varying linear (or affine) quadratic regulator
%
% in the following xbar = x-x0, ubar = u-u0
% @param Q can be a double matrix, a trajectory, or a 3x1 cell matrix
%   specifying the cost terms xbar'Q{1}xbar + xbar'Q{2} + Q{3}
% @param R can be a double matrix, a trajectory, or a 3x1 cell matrix
%   specifying the cost terms ubar'R{1}ubar + ubar'R{2} + R{3}
% @param Qf can be a double matrix, a 3x1 cell matrix, or a 
%   PolynomialLyapunovFunction (e.g. as would happen if you hand it back
%    Vtraj.eval(0) from a previous tvlqr trajectory).
%
% @option sqrtmethod @default true
% @option tspan @default utraj.getBreaks();
% @option N can be a double matrix or a trajectory, specifying additional
%    cost terms 2xbar'Nubar.  @default 0
%
% and purely for convenience:
% @option xdtraj adds terms so that the cost is 
%     (xbar - xbar_d)'*Q{1}*(xbar - xbar_d) + (xbar - xbar_d)'*Q{2}. @default 0
% @option udtraj adds terms so that the cost is 
%      (ubar - ubar_d)'*R{1}*(ubar - ubar_d) + (ubar - ubar_d)'*R{2}. @default 0
%
% in the following ybar = y-y0, where y0(t) = output(obj,t,x0(t),u0(t))
% @option Qy adds terms of the form ybar'*Qy{1}*ybar + ybar'*Qy{2} + Qy{3}. @default all zeros
% @option Ny adds terms of the form 2*ybar'*Ny*ubar.  @default 0.
% @option ydtraj adds terms of the form (ybar - ybar_d)'*Qy{1}*(ybar - ybar_d) + (ybar - ybar_d)'*Qy{2}. @default 0
%

if (nargin<7), options=struct(); end
if isfield(options,'tspan') 
  typecheck(options.tspan,'double');
else  
  options.tspan = utraj.getBreaks(); 
end
if ~isfield(options,'sqrtmethod'), options.sqrtmethod = true; end

if (~isCT(obj)), error('only handle CT case so far'); end

typecheck(xtraj,'Trajectory');
xtraj = inOutputFrame(xtraj,getStateFrame(obj));
typecheck(utraj,'Trajectory');
utraj = inOutputFrame(utraj,getInputFrame(obj));


nX = obj.getNumStates();
nU = obj.getNumInputs();
tspan=options.tspan;

if ~isfield(options,'N'), options.N=zeros(nX,nU); end

% create new coordinate frames
iframe = CoordinateFrame([obj.getStateFrame.name,' - x0(t)'],nX,obj.getStateFrame.prefix);
obj.getStateFrame.addTransform(AffineTransform(obj.getStateFrame,iframe,eye(nX),-xtraj));
iframe.addTransform(AffineTransform(iframe,obj.getStateFrame,eye(nX),xtraj));

oframe = CoordinateFrame([obj.getInputFrame.name,' + u0(t)'],nU,obj.getInputFrame.prefix);
oframe.addTransform(AffineTransform(oframe,obj.getInputFrame,eye(nU),utraj));
obj.getInputFrame.addTransform(AffineTransform(obj.getInputFrame,oframe,eye(nU),-utraj));

%  min_u(t) \int_0^T  x'Q{1}x + x'Q{2} + Q{3} + u'R{1}u + u'R{2} + R{3} + 2x'Nu
%    subject to xdot = Ax + Bu + c

if isa(Q,'double')
  sizecheck(Q,[nX,nX]);
  Q = {ConstantTrajectory(Q),ConstantTrajectory(zeros(nX,1)),ConstantTrajectory(0)};
elseif isa(Q,'Trajectory')
  Q = {Q,ConstantTrajectory(zeros(nX,1)),ConstantTrajectory(0)};  
elseif isa(Q,'cell')
  sizecheck(Q,3);
  
  if isa(Q{1},'double'), Q{1} = ConstantTrajectory(Q{1}); 
  else typecheck(Q{1},'Trajectory'); end
  sizecheck(Q{1},[nX,nX]);

  if isa(Q{2},'double'), Q{2} = ConstantTrajectory(Q{2});
  else typecheck(Q{2},'Trajectory'); end
  sizecheck(Q{2},[nX,1]);
  
  if isa(Q{3},'double'), Q{3} = ConstantTrajectory(Q{3});
  else typecheck(Q{3},'Trajectory'); end
  sizecheck(Q{3},1);
else
  error('Q must be a double, a trajectory, or a 3x1 cell array of doubles and trajectories');
end

if (isa(R,'double'))
  sizecheck(R,[nU,nU]);
  R = {ConstantTrajectory(R),ConstantTrajectory(zeros(nU,1)),ConstantTrajectory(0)};
elseif isa(R,'Trajectory')
  sizecheck(R,[nU,nU]);
  R = {R,ConstantTrajectory(zeros(nU,1)),ConstantTrajectory(0)};  
elseif isa(R,'cell')
  sizecheck(R,3);
  
  if isa(R{1},'double'), R{1} = ConstantTrajectory(R{1}); 
  else typecheck(R{1},'Trajectory'); end
  sizecheck(R{1},[nU,nU]);

  if isa(R{2},'double'), R{2} = ConstantTrajectory(R{2});
  else typecheck(R{2},'Trajectory'); end
  sizecheck(R{2},[nU,1]);
  
  if isa(R{3},'double'), R{3} = ConstantTrajectory(R{3});
  else typecheck(R{3},'Trajectory'); end
  sizecheck(R{3},1);  
else
  error('R must be a double, a trajectory, or a 3x1 cell array of doubles and trajectories');
end

if isa(Qf,'double')
  sizecheck(Qf,[nX,nX]);
  Qf = {Qf,zeros(nX,1),0};
elseif iscell(Qf)
  % intentionally left blank
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
if options.sqrtmethod && min(eig(Qf{1}))<eps
  error('sqrt method requires a strictly positive definite value for Qf{1}.  consider setting options.sqrtmethod=false if you want Qf{1}=0');
end

N=options.N;
if isa(N,'double')
  sizecheck(N,[nX,nU]);
  N = ConstantTrajectory(N);
elseif isa(N,'Trajectory')
  sizecheck(N,[nX,nU]);
else
  error('N must be a double or a trajectory');
end

% xdtraj adds terms so that the cost is (xbar - xbar_d)'*Q{1}*(xbar - xbar_d) + (xbar - xbar_d)'*Q{2}. @default 0
if isfield(options,'xdtraj')
  typecheck(options.xdtraj,'Trajectory');
  options.xdtraj = options.xdtraj.inFrame(iframe);
  
  Q{3} = Q{3} + options.xdtraj'*Q{1}*options.xdtraj - options.xdtraj'*Q{2};
  Q{2} = Q{2} - 2*Q{1}*options.xdtraj;
end

% udtraj adds terms so that the cost is (ubar - ubar_d)'*R{1}*(ubar - ubar_d) + (ubar - ubar_d)'*R{2}. @default 0
if isfield(options,'udtraj')
  typecheck(udtraj,'Trajectory');
  options.udtraj = options.udtraj.inFrame(oframe);
  
  R{3} = R{3} + options.udtraj'*R{1}*option.udtraj - options.udtraj'*R{2};
  R{2} = R{2} - 2*R{2}*options.udtraj;
end

nY = getNumOutputs(obj);

if isfield(options,'Qy') || isfield(options,'Ny')
  ts = unique([xtraj.getBreaks,utraj.getBreaks,options.tspan]);
  [y0traj,Ctraj,Dtraj] = getOutputTrajectories(obj,ts,xtraj,utraj,options);
end

% Qy adds terms of the form ybar'*Qy{1}*ybar + ybar'*Qy{2} + Qy{3}.  @default 0
if isfield(options,'Qy')
  Qy = options.Qy;
  if isa(Qy,'double')
    sizecheck(Qy,[nY,nY]);
    Qy = {ConstantTrajectory(Qy),ConstantTrajectory(zeros(nY,1)),ConstantTrajectory(0)};
  elseif isa(Qy,'Trajectory')
    Qy = {Qy,ConstantTrajectory(zeros(nY,1)),ConstantTrajectory(0)};
  elseif isa(Qy,'cell')
    sizecheck(Qy,3);
  
    if isa(Qy{1},'double'), Qy{1} = ConstantTrajectory(Qy{1});
    else typecheck(Qy{1},'Trajectory'); end
    sizecheck(Qy{1},[nY,nY]);
    
    if isa(Qy{2},'double'), Qy{2} = ConstantTrajectory(Qy{2});
    else typecheck(Qy{2},'Trajectory'); end
    sizecheck(Qy{2},[nY,1]);
    
    if isa(Qy{3},'double'), Qy{3} = ConstantTrajectory(Qy{3});
    else typecheck(Qy{3},'Trajectory'); end
    sizecheck(Qy{3},1);
  else
    error('Qy must be a double, a trajectory, or a 3x1 cell array of doubles and trajectories');
  end
else
  Qy=[];
end

% ydtraj adds terms of the form (ybar - ybar_d)'*Qy*(ybar - ybar_d) + (ybar - ybar_d)'*Qy. @default 0
if isfield(options,'ydtraj') && ~isempty(Qy)
  typecheck(options.ydtraj,'Trajectory');

  ybar_d = options.ydtraj.inFrame(getOutputFrame(obj)) - y0traj;
  Qy{3} = Qy{3} + ybar_d'*Qy{1}*ybar_d - ybar_d'*Qy{2};
  Qy{2} = Qy{2} - 2*Qy{1}*ybar_d;
end

if ~isempty(Qy)
  % add the y terms into Q, R, and N
  Q{1} = Q{1} + Ctraj'*Qy{1}*Ctraj;
  Q{2} = Q{2} + Ctraj'*Qy{2};
  Q{3} = Q{3} + Qy{3};
  R{1} = R{1} + Dtraj'*Qy{1}*Dtraj;
  R{2} = R{2} + Dtraj'*Qy{2};
  N = N + Ctraj'*Qy{1}*Dtraj;
end  

if isfield(options,'Ny') 
  Ny = options.Ny;
  typecheck(Ny,{'double','Trajectory'});
  sizecheck(Ny,[nY,nU]);
  
  N = N + Ctraj'*Ny;
  R{1} = R{1} + 2*Dtraj'*Ny;
end

xdottraj = fnder(xtraj);

if (options.sqrtmethod)
  Qf{1} = Qf{1}^(1/2);
end
S = cellODE(@ode45,@(t,S)affineSdynamics(t,S,obj,Q,R,N,xtraj,utraj,xdottraj,options),tspan(end:-1:1),Qf);
S = flipToPP(S);
if (options.sqrtmethod)
  S = recompS(S);
end
B = getBTrajectory(obj,S{1}.getBreaks(),xtraj,utraj,options);

% note that this returns what we would normally call -K.  here u(t) = u_0(t) + K(t) (x(t) - x_0(t)) 
K = affineKsoln(S,R,B,N);
  
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
   
  if isa(plant,'StochasticDrakeSystem')
   w0=zeros(1,plant.getNumDisturbances);
  [~,df] = geval(@plant.dynamics,ts(i),x0,u0,w0,options);
  else
  [~,df] = geval(@plant.dynamics,ts(i),x0,u0,options);   
  end
    
    
    
    B(:,:,i) = df(:,nX+1+(1:nU));
  end
  B = PPTrajectory(spline(ts,B));
end

function [y0,C,D] = getOutputTrajectories(plant,ts,xtraj,utraj,options)
  nX = length(xtraj); nU = length(utraj); nY = getNumOutputs(plant);
  y0 = zeros([nY,length(ts)]);
  C = zeros([nY,nX,length(ts)]);
  D = zeros([nY,nU,length(ts)]);
  for i=1:length(ts)
    x0=xtraj.eval(ts(i)); u0 = utraj.eval(ts(i));
    [f,df] = geval(@plant.output,ts(i),x0,u0,options);
    y0(:,i) = f;
    C(:,:,i) = df(:,1+(1:nX));
    D(:,:,i) = df(:,nX+1+(1:nU));
  end
  y0 = PPTrajectory(spline(ts,y0));
  C = PPTrajectory(spline(ts,C));
  D = PPTrajectory(spline(ts,D));
end

function Sdot = Sdynamics(t,S,plant,Qtraj,Rtraj,xtraj,utraj,xdottraj,options)
  x0 = xtraj.eval(t); u0 = utraj.eval(t);
  Q = Qtraj.eval(t); Ri = inv(Rtraj.eval(t));
  nX = length(x0); nU = length(u0);
  [f,df] = geval(@plant.dynamics,t,x0,u0,options);
  A = df(:,1+(1:nX));
  B = df(:,nX+1+(1:nU));
  Sdot = -(Q - S*B*Ri*B'*S + S*A + A'*S);
  if (min(eig(S))<0) warning('S is not positive definite'); end
end

function K = Ksoln(S,Ri,B,N)
  K = -Ri*B'*S;
end

function Sdot = affineSdynamics(t,S,plant,Qtraj,Rtraj,Ntraj,xtraj,utraj,xdottraj,options)
  % see doc/derivations/tvlqr-latexit.pdf 

  x0 = xtraj.eval(t); u0 = utraj.eval(t); xdot0 = xdottraj.eval(t);

  Q{1}=Qtraj{1}.eval(t); Q{2}=Qtraj{2}.eval(t); Q{3}=Qtraj{3}.eval(t); 
  R{1}=Rtraj{1}.eval(t); R{2}=Rtraj{2}.eval(t); R{3}=Rtraj{3}.eval(t);
  Ri = inv(R{1});
  N = Ntraj.eval(t);
  
  nX = length(x0); nU = length(u0);

  if isa(plant,'StochasticDrakeSystem')
   w0=zeros(1,plant.getNumDisturbances);
  [xdot,df] = geval(@plant.dynamics,t,x0,u0,w0,options);
  else
  [xdot,df] = geval(@plant.dynamics,t,x0,u0,options);    
  end
  A = df(:,1+(1:nX));
  B = df(:,nX+1+(1:nU));
  c = xdot - xdot0;
  
  if (options.sqrtmethod)
    ss = inv(S{1}');
    Sdot{1} = -.5*Q{1}*ss - A'*S{1} + .5*(N+S{1}*S{1}'*B)*Ri*(B'*S{1}+N'*ss);  % sqrt method  %#ok<MINV>
    Sorig = S{1}*S{1}';
  else
    Sdot{1} = -(Q{1} - (N+S{1}*B)*Ri*(B'*S{1}+N') + S{1}*A + A'*S{1});
    Sorig = S{1};
  end
  if (min(eig(Sorig))<0) 
    warning('Drake:TVLQR:NegativeS','S is not positive definite'); 
  end
  
  rs = (R{2}+B'*S{2})/2;
  Sdot{2} = -(Q{2} - 2*(N+Sorig*B)*Ri*rs + A'*S{2} + 2*Sorig*c);
  Sdot{3} = -(Q{3}+R{3} - rs'*Ri*rs + S{2}'*c);  

%  plot(t,S{2}(1),'r.');
%   if (9.95<=t & t<10)
%     t
%     disp(['Q{1} = ',mat2str(Q{1})]);
%     disp(['Q{2} = ',mat2str(Q{2})]);
%     disp(['Q{3}+R{3} = ',mat2str(Q{3}+R{3})]);
%     disp(['R{1} = ',mat2str(R{1})]);
%     disp(['R{2} = ',mat2str(R{2})]);
% %    N
% %    A
% %    B
% %    c
%     disp(['S{1} = ',mat2str(S{1})]);
%     disp(['S{2} = ',mat2str(S{2})]);
%     disp(['S{3} = ',mat2str(S{3})]);
%     disp(['Sdot{1} = ',mat2str(Sdot{1})]);
%     disp(['Sdot{2} = ',mat2str(Sdot{2})]);
%     disp(['Sdot{3} = ',mat2str(Sdot{3})]);
%   end
  
end
    
function S = recompS(Ssqrt)
  S{1} = Ssqrt{1}*Ssqrt{1}';
  S{2} = Ssqrt{2};
  S{3} = Ssqrt{3};
end

function K = affineKsoln(S,R,B,N)
  Ri = inv(R{1});
  K{1} = -Ri*(B'*S{1}+N');
  K{2} = -.5*Ri*(B'*S{2} + R{2});
end

