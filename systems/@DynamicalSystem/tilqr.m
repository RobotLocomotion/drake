function ltisys = tilqr(obj,x0,u0,Q,R)

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
  error('i''ve assumed C=I,D=0 so far.');
end

ltisys = LTIControl(x0,u0,K,S);

end

