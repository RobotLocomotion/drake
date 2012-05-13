function [ltisys,V] = tilqr(obj,x0,u0,Q,R)
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
% @retval V an msspoly representation of the cost-to-go function x'*S*x

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


ltisys = TimeInvariantLinearSystem([],[],[],[],[],K);
if (all(x0==0))
  ltisys.input_frame = obj.state_frame;
else
  ltisys.input_frame = CoordinateFrame([obj.state_frame.name,' - ', mat2str(x0,2)],length(x0));
  obj.state_frame.addTransform(AffineTransform(obj.state_frame,ltisys.input_frame,eye(size(x0)),-x0));
  ltisys.input_frame.addTransform(AffineTransform(ltisys.input_frame,obj.state_frame,eye(size(x0)),+x0));
end
if (all(u0==0))
  ltisys.output_frame = obj.input_frame;
else
  ltisys.output_frame = CoordinateFrame([obj.input_frame.name,' + ',mat2str(u0,2)],length(u0));
  ltisys.output_frame.addTransform(AffineTransform(ltisys.output_frame,obj.input_frame,eye(size(u0)),u0));
  obj.input_frame.addTransform(AffineTransform(obj.input_frame,ltisys.output_frame,eye(size(u0)),-u0));
end

if (nargout>1)
  x=ltisys.input_frame.poly; %msspoly('x',getNumStates(obj));
  V=PolynomialLyapunovFunction(ltisys.input_frame,x'*S*x);
end

end

