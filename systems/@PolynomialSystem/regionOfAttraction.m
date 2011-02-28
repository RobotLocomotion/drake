function V = regionOfAttraction(sys,x0,V0,options)
% estimates the region of attraction, defined as the interior region for
% the level-set V<=1 surrounding the origin.  V0 is the initial guess for V.  
% Both V0 and V are specified relative to x0 (e.g. V(0) is the value of the
% Lyapunov function at x0).  

if (~isCT(sys)) error('only handle CT case so far'); end
if (~isTI(sys)) error('only for TI systems so far'); end

typecheck(x0,'double');
sizecheck(x0,[getNumStates(sys),1]);

% check fixed point
% check Hessian Vdot at origin, to make sure it's negative def. 

x = sys.p_x;
% f is time-invariant zero-input dynamics relative to x0
f = subss(sys.p_dynamics,[sys.p_t;x;sys.p_u],[0;x - x0;zeros(sys.num_u,1)]);

if (nargin<3) 
  A = double(subs(diff(f,x),x,zeros(sys.num_x,1)));
  Q = eye(sys.num_x);  % todo: take this as a parameter?
  P = lyap(A',Q);
  V = x'*P*x;
else
  V = V0;
end

typecheck(V,'msspoly');
if any([V.m V.n]~=1) error('V0 must be a scalar msspoly'); end

Vdot = diff(V,x)*f;
Lmonom = monomials(x,0:deg(Vdot,x));  % just a guess (todo: take as a parameter)

prog = mssprog;

rho = msspoly('r');
prog.free = rho;
    
[prog,l] = new(prog,length(Lmonom),'free');
L = l'*Lmonom;
    
prog.sos = (x'*x)*(V - rho) +  L*Vdot;
prog.sedumi = -rho;
    
rho = double(prog(rho))
if (rho<=0) error('optimization failed'); end

V = V/rho;

end