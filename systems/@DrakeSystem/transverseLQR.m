function [ltvsys,Vtraj] = transverseLQR(obj,xtraj,utraj,Q,R,Qf,transSurf)
% Compute LQR control for transversal system
%
% irm@mit.edu

nX = getNumStates(obj);
nU = getNumInputs(obj);

typecheck(xtraj,'Trajectory');
sizecheck(xtraj,[nX,1]);
typecheck(utraj,'Trajectory');
sizecheck(utraj,[nU,1]);

tspan=utraj.getBreaks();

if (isa(Q,'double'))
  Q = PPTrajectory(zoh(tspan([1,end]),repmat(Q,[1 1 2])));
end
if (isa(R,'double'))
  R = PPTrajectory(zoh(tspan([1,end]),repmat(R,[1 1 2])));
end

Pi = transSurf.getPi(tspan(end));

typecheck(Q,'Trajectory');  
sizecheck(Q,[nX,nX]);
typecheck(R,'Trajectory');
sizecheck(R,[nU,nU]);
typecheck(Qf,'double');
if (all(size(Qf)==[nX nX]))
  Qf = Pi*Qf*Pi';
end
sizecheck(Qf,[nX-1,nX-1]);

if (~isCT(obj)), error('only handle CT case so far'); end

Ssol = matrixODE(@ode45,@(t,S)Sdynamics(t,S,obj,Q,R,xtraj,utraj,transSurf),tspan(end:-1:1),Qf);
Ksol = FunctionHandleTrajectory(@(t)Ksoln(t,obj,Ssol,R,xtraj,utraj,transSurf),[nX-1, nU],tspan);
Sdot = FunctionHandleTrajectory(@(t)Sdynamics(t,Ssol.eval(t),obj,Q,R,xtraj,utraj,transSurf),[nX-1, nX-1],tspan);

ltvsys = TransverseLinearControl(xtraj,utraj,Ksol,transSurf,Ssol,Sdot,Q,R,obj);

if (nargout>1)
  checkDependency('spotless');
  xperp=msspoly('p',nX-1);
  tau=msspoly('t',1);
  Vtraj = PolynomialTrajectory(@(t) (xperp-getPi(transSurf,t)*xtraj.eval(t))'*(Ssol.eval(t)+Sdot.eval(t)*(tau-t))*(xperp-getPi(transSurf,t)*xtraj.eval(t)),tspan);
end

   
end


  function Sdot = Sdynamics(t,S,plant,Qtraj,Rtraj,xtraj,utraj,transSurf)
    x0 = xtraj.eval(t); u0 = utraj.eval(t);
    Q = Qtraj.eval(t); Ri = inv(Rtraj.eval(t));
    nX = length(x0); nU = length(u0);
    [ft,df] = geval(@plant.dynamics,t, x0, u0);
    A = df(:,1+(1:nX));
    B = df(:,nX+1+(1:nU));
    
    zt = transSurf.z.eval(t);
    zdott = transSurf.zdot.eval(t);
    
    [Pi, Pidot] = transSurf.getPi(t);
    
    % derivatives of tau dynamics wrt x_perp and u
        
    dtau_dxperp = (zt'*A*Pi'+zdott'*Pi')/(zt'*ft);
    dtau_du = zt'*B/(zt'*ft);
        
    % transverse A, B and Q
    Atr = Pidot*Pi' + Pi*A*Pi' - Pi*ft*dtau_dxperp;
    Btr = Pi*B-Pi*ft*dtau_du;
        
    Qtr = Pi*Q*Pi';
    
    Sdot = -(Qtr - S*Btr*Ri*Btr'*S + S*Atr + Atr'*S);  %#ok<MINV>
  end
      
  function K = Ksoln(t,plant,Straj,Rtraj,xtraj,utraj, transSurfi)
    S = Straj.eval(t); R = Rtraj.eval(t); x0=xtraj.eval(t); u0 = utraj.eval(t);
    nX = length(x0); nU = length(u0);
    [ft,df] = geval(@plant.dynamics,t, x0, u0);
    B = df(:,nX+1+(1:nU));
    zt = transSurfi.z.eval(t);
    [Pi, ~] = transSurfi.getPi(t);
    
    % derivatives of tau dynamics wrt x_perp and u
        
    dtau_du = zt'*B/(zt'*ft);
    
    Btr = Pi*B-Pi*ft*dtau_du;
    K = R\(Btr'*S);
  end
