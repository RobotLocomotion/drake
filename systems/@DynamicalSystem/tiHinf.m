function [ltisys,V] = tiHinf(obj,x0,u0,Q,R,Bw,gamma)
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

typecheck(x0,'double');
sizecheck(x0,[nX,1]);
typecheck(u0,'double');
sizecheck(u0,[nU,1]);
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

ltisys = LTIControl(x0,u0,K);
ltisys = setAngleFlags(ltisys,obj.output_angle_flag,[],obj.input_angle_flag);

if (nargout>1)
  x=msspoly('x',getNumStates(obj));
  V = (x-x0)'*S*(x-x0);
end

end

