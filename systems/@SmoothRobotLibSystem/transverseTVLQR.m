function ltvsys = transverse_tvlqr(obj,xtraj,utraj,Q,R,Qf,TransSurf)
% Compute LQR control for transversal system
%
% irm@mit.edu

typecheck(xtraj,'Trajectory');
sizecheck(xtraj,[getNumStates(obj),1]);
typecheck(utraj,'Trajectory');
sizecheck(utraj,[getNumInputs(obj),1]);

nX = xtraj.dim;
nU = utraj.dim;
tspan=utraj.getBreaks();

if (isa(Q,'double'))
  Q = PPTrajectory(zoh(tspan([1,end]),repmat(Q,[1 1 2])));
end
if (isa(R,'double'))
  R = PPTrajectory(zoh(tspan([1,end]),repmat(R,[1 1 2])));
end

[Pi, Pidot] = TransSurf.getPi(tspan(end));

typecheck(Q,'Trajectory');  
sizecheck(Q,[nX,nX]);
typecheck(R,'Trajectory');
sizecheck(R,[nU,nU]);
typecheck(Qf,'double');
sizecheck(Qf,[nX,nX]);

if (~isCT(obj)) error('only handle CT case so far'); end

Ssol = matrixODE(@ode45,@(t,S)Sdynamics(t,S,obj,Q,R,xtraj,utraj),tspan(end:-1:1),Pi*Qf*Pi');
Ksol = FunctionHandleTrajectory(@(t)Ksoln(t,obj,Ssol,R,xtraj,utraj,TransSurf),[nX nU],tspan);
Sdot = FunctionHandleTrajectory(@(t)Sdynamics(t,Ssol.eval(t),obj,Q,R,xtraj,utraj),[nX nX],tspan);

  function Sdot = Sdynamics(t,S,plant,Qtraj,Rtraj,xtraj,utraj)
    x0 = xtraj.eval(t); u0 = utraj.eval(t);
    Q = Qtraj.eval(t); Ri = inv(Rtraj.eval(t));
    nX = length(x0); nU = length(u0);
    [ft,df] = geval(@plant.dynamics,t, x0, u0);
    A = df(:,1+(1:nX));
    B = df(:,nX+1+(1:nU));
    
    zt = TransSurf.z.eval(t);
    zdott = TransSurf.zdot.eval(t);
    
    [Pi, Pidot] = TransSurf.getPi(t);
    
    % derivatives of tau dynamics wrt x_perp and u
        
    dtau_dxperp = (zt'*A*Pi'+zdott'*Pi')/(zt'*ft);
    dtau_du = zt'*B/(zt'*ft);
        
    % transverse A, B and Q
    Atr = Pidot*Pi' + Pi*A*Pi' - Pi*ft*dtau_dxperp;
    Btr = Pi*B-Pi*ft*dtau_du;
        
    Qtr = Pi*Q*Pi';
        
    
    Sdot = -(Qtr - S*Btr*Ri*Btr'*S + S*Atr + Atr'*S);
  end
      
  function K = Ksoln(t,plant,Straj,Rtraj,xtraj,utraj, TransSurfi)
    S = Straj.eval(t); Ri = inv(Rtraj.eval(t)); x0=xtraj.eval(t); u0 = utraj.eval(t);
    nX = length(x0); nU = length(u0);
    [ft,df] = geval(@plant.dynamics,t, x0, u0);
    B = df(:,nX+1+(1:nU));
    zt = TransSurfi.z.eval(t);
    [Pi, ~] = TransSurfi.getPi(t);
    
    % derivatives of tau dynamics wrt x_perp and u
        
    dtau_du = zt'*B/(zt'*ft);
    
    Btr = Pi*B-Pi*ft*dtau_du;
    K = Ri*Btr'*S;
  end
    
ltvsys = TransverseLinearControl(xtraj,utraj,Ksol,TransSurf,Ssol,Sdot);

end