function [ltisys,Vcandidate] = tilqr(obj,x0,u0,Q,R,options)
% Computes an LQR controller to stabilize the system around (x0,u0)
%
% Linearizes the system around the nominal point then calls lqr (if the
% input and output sample times of the model are continuous) otherwise
% calls dlinearize/dlqr.
%
% The system must be time-invariant.
%
% @param x0 the nominal/goal state (a Point)
% @param u0 the nominal input (a Point)
% @param Q,R describe the LQR cost function \int dt (x'*Q*x + u'*R*u)
% @option angle_flag boolean vector the same size as x which causes the
% resulting controller to wrap around 2pi for the dimensions where
% angle_flag is true. @default false
% @option N adds the off-diagonal terms to the cost function
%   + \int dt (2*x'*N*u). @default N=0
% @option Qy adds terms to the cost function specified via the output
%   + \int dt ( y'*Qy*y ). @default Qy=0
%
% @retval ltisys a system which implements the control u=-K*x
% @retval Vcandidate an msspoly representation of the cost-to-go function x'*S*x

% ts is a simulink sample time
if (~isTI(obj)) error('I don''t know that this system is time invariant.  Set the TI flags and rerun this method if you believe the system to be.'); end

if isa(x0,'Point')
  x0 = double(x0.inFrame(obj.getStateFrame));
end
if isa(u0,'Point')
  u0 = double(u0.inFrame(obj.getInputFrame));
end

if (nargin<6) options=struct(); end
if ~isfield(options,'angle_flag') options.angle_flag = false; end
if ~isfield(options,'N') options.N = 0*x0*u0'; end

ts = max(getInputSampleTimes(obj),getOutputSampleTimes(obj));

tol = 1e-6;
if (ts(1)==0) % then it's CT
  [A,B,C,D,xdot0] = linearize(obj,0,x0,u0);
  if (any(abs(xdot0)>tol))
    xdot0
    error('(x0,u0) is not a fixed point');
  end
else
  [A,B,C,D,xn0] = dlinearize(obj,ts(1),0,x0,u0);
  if (any(abs(xn0-x0)>tol))
    xn0
    error('(x0,u0) is not a fixed point');
  end
end

if isfield(options,'Qy')
  % see 'doc lqry'
  Q = Q + C'*options.Qy*C;
  R = R + D'*options.Qy*D;
  options.N = options.N + C'*options.Qy*D;
end

if getNumUnilateralConstraints(obj)>0,
  error('no unilateral constraints allowed');
end

if getNumStateConstraints(obj)>0
  % if there are state constraints, the system will be uncontrollable
  % in the full coordinates, so project down to the unconstrained
  % coordinates:
  %
  % xdot = Ax + Bu, Fx = 0
  %
  % P := null(F)   (so FP = 0)
  % F is d-by-n,  P is n-by-(n-d)
  % z := P'x , P'P = I,  x = Pz
  % x'Qx = z P' Q P z , x'Nu = z'P'Nu
  % zdot = P'xdot = P'Ax + P'Bu = P'APz + P'B u
  %
  % u = -Kz = -KP'x, J = z'Sz = x'PSP'x
  %
  % it also works (the same!) for discrete time
  % xn = Ax + Bu
  % zn = P'xn = P'Ax + P'B u = P'APz + P'B u

  [phi,F] = geval(@obj.stateConstraints,x0);
  if ~valuecheck(phi,0,1e-6)
    phi
    error('Drake:TILQR:UnsatisfiedStateContraint','The system has state constraints which are not satisfied at x0');
  end
  P = null(full(F));
  A = P'*A*P;
  B = P'*B;
  Q = P'*Q*P;
  options.N = P'*options.N;
end

if (ts(1)==0) % then it's CT
  [K,S] = lqr(full(A),full(B),Q,R,options.N);
else
  [K,S] = dlqr(A,B,Q,R,options.N);
end

if getNumStateConstraints(obj)>0
  % project the result back to the full state
  K = K*P';
  S = P*S*P';
end

ltisys = LinearSystem([],[],[],[],[],-K);
if (all(x0==0))
  ltisys = setInputFrame(ltisys,obj.getStateFrame);
else
  ltisys = setInputFrame(ltisys,CoordinateFrame([obj.getStateFrame.name,' - ', mat2str(x0,3)],length(x0),obj.getStateFrame.prefix));
  obj.getStateFrame.addTransform(AffineTransform(obj.getStateFrame,ltisys.getInputFrame,eye(length(x0)),-x0));
  ltisys.getInputFrame.addTransform(AffineTransform(ltisys.getInputFrame,obj.getStateFrame,eye(length(x0)),+x0));
end
if any(options.angle_flag)
  ltisys = setInputFrame(ltisys,ltisys.getInputFrame().constructFrameWithAnglesWrapped(options.angle_flag));
  if (nargout>1)
    warning('DynamicalSystem:TILQR:NonSmoothLyapunovFunction','Constructing a wrapped (around 2*pi) frame for the controller and Lyapunov function. This may interfere with smooth analysis using the Lyapunov function, in which case you may prefer to defer this construction til post-analysis by unsetting options.angle_flag');
  end
end
if (all(u0==0))
  ltisys = setOutputFrame(ltisys,obj.getInputFrame);
else
  ltisys = setOutputFrame(ltisys,CoordinateFrame([obj.getInputFrame.name,' + ',mat2str(u0,3)],length(u0),obj.getInputFrame.prefix));
  ltisys.getOutputFrame.addTransform(AffineTransform(ltisys.getOutputFrame,obj.getInputFrame,eye(length(u0)),u0));
  obj.getInputFrame.addTransform(AffineTransform(obj.getInputFrame,ltisys.getOutputFrame,eye(length(u0)),-u0));
end

if (nargout>1)
  Vcandidate=QuadraticLyapunovFunction(ltisys.getInputFrame,S);
end

end
