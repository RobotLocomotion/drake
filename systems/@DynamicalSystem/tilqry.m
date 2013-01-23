function varargout = tilqry(obj,x0,u0,Q,R,options)
% calls tilqr to minimize the objective
%  min_u \int_0^\inf  [ y^T Q y + u^T R u ] dt
% (or the DT equivalent)

if (~isTI(obj)) error('I don''t know that this system is time invariant.  Set the TI flags and rerun this method if you believe the system to be.'); end
typecheck(x0,'Point');
typecheck(u0,'Point');

typecheck(Q,'numeric');
sizecheck(Q,[getNumOutputs(obj),getNumOutputs(obj)]);
typecheck(R,'numeric');
sizecheck(R,[getNumInputs(obj),getNumInputs(obj)]);

x0 = x0.inFrame(obj.getStateFrame);
u0 = u0.inFrame(obj.getInputFrame);

ts = max(getInputSampleTimes(obj),getOutputSampleTimes(obj));
if (ts(1)==0) % then it's CT
  [A,B,C,D] = linearize(obj,0,double(x0),double(u0));
else
  [A,B,C,D] = dlinearize(obj,ts(1),0,double(x0),double(u0));
end

if (nargin<6) options=struct(); end
if isfield(options,'N') error('N is not supported here yet'); end

% see 'doc lqry'
R = R + D'*Q*D;
options.N = C'*Q*D;
Q = C'*Q*C;

varargout = cell(1,nargout);
[varargout{:}] = tilqr(obj,x0,u0,Q,R,options);
