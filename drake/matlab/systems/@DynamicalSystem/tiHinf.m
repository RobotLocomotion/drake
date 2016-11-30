function [ltisys,Vcandidate] = tiHinf(obj,x0,u0,Q,R,Bw,gamma)
% Suboptimal state-feedback H-infinity controller for LTI system with bound gamma.
%
% For continuous systems, the design is based on the dynamics
%   xdot = A*x + B*u + Bw*w 
% where w is unit-covariance Gaussian white noise.  
%
% The resulting controller is the result of
%   min_u  max_w  \int_0^inf (x^T Q x + u^T R u - \gamma w^T w) dt
%
% @param x0 linearize the dynamics of obj around this operating point (state)
% @param u0 linearize the dynamics of obj around this operating point (input)
% @param Q pos def matrix that describes the cost on state
% @param R pos def matrix that describes the cost on input
% @param Bw the disturbance input matrix
% &param gamma gain on disturbance cost
%

if (~isTI(obj)) error('I don''t know that this system is time invariant.  Set the TI flags and rerun this method if you believe the system to be.'); end
if (~isCT(obj)) error('Only implemented for CT systems so far'); end

nX=getNumStates(obj);
nU=getNumInputs(obj);
nW=size(Bw,2);

typecheck(x0,'Point');
x0 = double(x0.inFrame(obj.getStateFrame));
typecheck(u0,'Point');
u0 = double(u0.inFrame(obj.getInputFrame));
typecheck(Q,'double');
sizecheck(Q,[nX nX]);
typecheck(R,'double');
sizecheck(R,[nU nU]);
typecheck(Bw,'double');
sizecheck(Bw,[nX nW]);
typecheck(gamma,'double');
sizecheck(gamma,[1 1]);

[A,B,C,D,xdot0] = linearize(obj,0,x0,u0);
tol = 1e-10;
if (any(any(abs(C-eye(nX))>tol)) || any(abs(D(:))>tol))
  C
  D
  error('i''ve assumed C=I,D=0 so far.');
end
if (any(abs(xdot0)>tol))
  xdot0
  error('(x0,u0) is not a fixed point');
end

S = care(A,[B,Bw],Q,blkdiag(R,-gamma^2*eye(nW)));
K=inv(R)*B'*S;  
if (isempty(S)) 
  error('L2 gain gamma is infeasible.  Try increasing gamma.');
end

if (obj.getStateFrame ~= obj.getOutputFrame)  % todo: remove this or put it in a better place when I start doing more observer-based designs
  warning('designing full-state feedback controller but plant has different output frame than state frame'); 
end

  
ltisys = LinearSystem([],[],[],[],[],-K);
if (all(x0==0))
  ltisys = setInputFrame(ltisys,obj.getStateFrame);
else
  ltisys = setInputFrame(ltisys,CoordinateFrame([obj.getStateFrame.name,' - ', mat2str(x0,3)],length(x0),obj.getStateFrame.prefix));
  obj.getStateFrame.addTransform(AffineTransform(obj.getStateFrame,ltisys.getInputFrame,eye(length(x0)),-x0));
  ltisys.getInputFrame.addTransform(AffineTransform(ltisys.getInputFrame,obj.getStateFrame,eye(length(x0)),+x0));
end
if (all(u0==0))
  ltisys = setOutputFrame(ltisys,obj.getInputFrame);
else
  ltisys = setOutputFrame(ltisys,CoordinateFrame([obj.getInputFrame.name,' + ',mat2str(u0,3)],length(u0),obj.getInputFrame.prefix));
  ltisys.getOutputFrame.addTransform(AffineTransform(ltisys.getOutputFrame,obj.getInputFrame,eye(length(u0)),u0));
  obj.getInputFrame.addTransform(AffineTransform(obj.getInputFrame,ltisys.getOutputFrame,eye(length(u0)),-u0));
end

if (nargout>1)
  x=ltisys.getInputFrame.getPoly; %msspoly('x',getNumStates(obj));
  Vcandidate=QuadraticLyapunovFunction(ltisys.getInputFrame,S);
end

end

