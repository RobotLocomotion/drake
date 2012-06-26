function [ltisys,Vcandidate] = tilqr(obj,x0,u0,Q,R)
% Computes an LQR controller to stabilize the system around (x0,u0)
%
% Linearizes the system around the nominal point then calls lqr (if the
% input and output sample times of the model are continuous) otherwise
% calls dlinearize/dlqr.
%
% The system must be time-invariant.
%
% @param x0 the nominal/goal state
% @param u0 the nominal input
% @param Q,R describe the LQR cost function \int dt (x'*Q*r + u'*R*u)
%
% @retval ltisys a system which implements the control u=-K*x
% @retval Vcandidate an msspoly representation of the cost-to-go function x'*S*x

% ts is a simulink sample time
if (~isTI(obj)) error('I don''t know that this system is time invariant.  Set the TI flags and rerun this method if you believe the system to be.'); end
typecheck(x0,'double');
sizecheck(x0,[getNumStates(obj),1]);
typecheck(u0,'double');
sizecheck(u0,[getNumInputs(obj),1]);

ts = max(getInputSampleTimes(obj),getOutputSampleTimes(obj));

tol = 1e-10;
if (ts(1)==0) % then it's CT
  [A,B,C,D,xdot0] = linearize(obj,0,x0,u0);
  if (any(abs(xdot0)>tol))
    xdot0
    error('(x0,u0) is not a fixed point');
  end
  [K,S] = lqr(full(A),full(B),Q,R);
else
  [A,B,C,D,xn0] = dlinearize(obj,ts(1),0,x0,u0);
  if (any(abs(xn0-x0)>tol))
    xn0
    error('(x0,u0) is not a fixed point');
  end
  [K,S] = dlqr(A,B,Q,R);
end
if (any(any(abs(C-eye(getNumStates(obj)))>tol)) || any(abs(D(:))>tol))
  C
  D
  warning('i''ve assumed C=I,D=0 so far.');
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
  x=ltisys.getInputFrame.poly; %msspoly('x',getNumStates(obj));
  Vcandidate=PolynomialLyapunovFunction(ltisys.getInputFrame,x'*S*x);
end

end

