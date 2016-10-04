function [c,Ktraj,Straj,Ptraj,Btraj,Ftraj,Straj_full] = constrainedtvlqr(obj,xtraj,utraj,Q,R,Qf,options)
%CONSTRAINEDTVLQR
% TVLQR with constraints
% @input obj The plant object
% @input xtraj Nominal state trajectory
% @input utraj Nominal input trajectory
% @input Q 
% @input R
% @input Qf
% @input options.tspan time span, defaults to using the xtraj tspan
% @input options.P0 Optionally, supply P0
% Performs time varying lqr on a reduced order model
% @return c The controller object
% @return Ktraj control gains
% @return Straj quadratic cost term
% @return Ptraj Orthonormal basis of controllable states
% @return Btraj B(t), where xdot = A(t)x + B(t)u

if nargin < 7
  options = struct();
end

if isfield(options,'tspan')
  typecheck(options.tspan,'double');
  tspan = options.tspan;
else
  tspan = xtraj.tspan;
end

if isfield(options,'alpha_1')
  typecheck(options.alpha_1,'double');
  alpha_1 = options.alpha_1;
else
  alpha_1 = 1000; % was 5000
end

if isfield(options,'alpha_2')
  typecheck(options.alpha_2,'double');
  alpha_2 = options.alpha_2;
else
  alpha_2 = 1;
end

if ~isfield(options,'sqrtmethod')
  options.sqrtmethod = true;
end

x0 = xtraj.eval(tspan(1));
u0 = utraj.eval(tspan(1));
xf = xtraj.eval(tspan(2));
uf = utraj.eval(tspan(2));


% if isfield(options,'P0')
%   P0 = options.P0;
%   if any(abs(F0*P0) > 1e-2)
%     error('F0*P0 should be close to 0')
%   end  
% else
  
% end

[phi0,J0] = obj.positionConstraints(x0(1:obj.getNumPositions));

F0 = getFandFdot(obj,tspan(1),x0,u0);
Ff = getFandFdot(obj,tspan(2),xf,uf);


dynamicsFun = @(t,x,u) constrainedDynamics(obj,t,x,u);

nQ = obj.getNumPositions();                         %unconstrained position state size
nQC = length(phi0);          %number of constrained positions


if false || ~options.sqrtmethod
  P0 = null(F0);

display(sprintf('Solving for P forwards, from %f to %f',tspan(1),tspan(end)));
Pfun = @(t,P) Pdynamics(obj,t,xtraj,utraj,P,alpha_1,alpha_2);
[tout,yout] = ode45(Pfun,tspan,P0);


Ptraj = PPTrajectory(foh(tout,reshape(yout',2*nQ,2*nQ-2*nQC,[])));
Pf = Ptraj.eval(tspan(2));

Sf = Pf'*Qf*Pf;
if ~options.sqrtmethod
  Sfun = @(t,S) Sdynamics(t,S,obj,dynamicsFun,xtraj,utraj,Ptraj,Q,R,alpha_1,alpha_2);
else
  Sf = Sf^(1/2);
  Sfun = @(t,S) sqrtSdynamics(t,S,obj,dynamicsFun,xtraj,utraj,Ptraj,Q,R,alpha_1,alpha_2);
end
display(sprintf('Solving for S backwards, from %f to %f',tspan(end),tspan(1)));

[tout,yout] = ode45(Sfun,tspan(end:-1:1),Sf);

S_data = reshape(yout',2*nQ - 2*nQC,2*nQ - 2*nQC,[]);
Straj = PPTrajectory(foh(tout(end:-1:1),S_data(:,:,end:-1:1)));

if options.sqrtmethod
  Straj = Straj*Straj';
end

else
  Pf = null(Ff);
  Sf = Pf'*Qf*Pf;
  Sf = real(Sf^(1/2));
  display(sprintf('Solving for P and S backwards, from %f to %f',tspan(end),tspan(1)));
  PandSfun = @(t,PandSqrtS) PandSqrtSdynamics(t,PandSqrtS,obj,dynamicsFun,xtraj,utraj,Q,R,-alpha_1,-alpha_2,numel(Pf));
  [tout,yout] = ode45(PandSfun,tspan(end:-1:1),[Pf(:);Sf(:)]);

  P_data = reshape(yout(:,1:numel(Pf))',2*nQ,2*nQ-2*nQC,[]);
  Ptraj = PPTrajectory(foh(tout(end:-1:1),P_data(:,:,end:-1:1)));
  S_data = reshape(yout(:,numel(Pf)+1:end)',2*nQ - 2*nQC,2*nQ - 2*nQC,[]);
  Straj = PPTrajectory(foh(tout(end:-1:1),S_data(:,:,end:-1:1)));
  Straj = Straj*Straj';
end

Btraj = getBTrajectory(obj,Straj.getBreaks(),xtraj,utraj);

Ktraj = inv(R)*(Ptraj'*Btraj)'*Straj*Ptraj';

c = AffineSystem([],[],[],[],[],[],[],-Ktraj,Ktraj*xtraj + utraj);
c = c.setInputFrame(obj.getOutputFrame);
c = c.setOutputFrame(obj.getInputFrame);

Straj_full = Ptraj*Straj*Ptraj';

if size(F0,1)>0
  tt = Straj.pp.breaks;
  F_data = zeros(size(F0,1),size(F0,2),length(tt));
  for i=1:length(tt),
    F_data(:,:,i) = getFandFdot(obj,tt(i),xtraj.eval(tt(i)),utraj.eval(tt(i)));
  end
  Ftraj = PPTrajectory(foh(tt,F_data));
else
  Ftraj = [];
end
end

function B = getBTrajectory(p,ts,xtraj,utraj)
  nX = length(xtraj); nU = length(utraj);
  B = zeros([nX,nU,length(ts)]);
  dynamicsFun = @(t,x,u) constrainedDynamics(p,t,x,u);
  for i=1:length(ts)
    x0=xtraj.eval(ts(i)); 
    u0 = utraj.eval(ts(i));    
    [xd,dxd] = dynamicsFun(ts(i),x0,u0);
    B(:,:,i) = dxd(:,nX+1+(1:nU));
  end
  B = PPTrajectory(spline(ts,B));
end

  function [F,Fdot]=getFandFdot(obj,t,x,u)
  
    q = x(1:obj.getNumPositions());
    qd = x(obj.getNumPositions()+1:end);
    
    % check if this doesn't calculate the gradient
%     [~,J,dJ] = obj.positionConstraints(q);
    [phi,J,dJ,Jdotqd,dJdotqd] = obj.positionConstraintslWithJdot(q,qd);
    dJ = reshape(dJ,numel(J),[]);
    if isempty(J)
      F = zeros(0,length(x));
      Fdot = F;
      return;
    end
    Jdot = reshape(dJ * qd, size(J));
    
    F = full([J zeros(size(J));Jdot J]);
    
    if nargout > 1
      xd = obj.constrainedDynamics(t,x,u);
      %       [xd,A] = obj.constrainedDynamics(t,x,u);
      %       Fdot = - F*A(:,2:1+length(x));
        Jddot = dJdotqd(:,1:obj.getNumPositions) + matGradMult(dJ,xd(obj.getNumPositions+1:end));
        Fdot = [Jdot zeros(size(Jdot)); Jddot Jdot];
    end
  end


function Pdot = getPdot(obj,t,x,u,P,alpha_1,alpha_2)

  n = length(x);
  d = n - size(P,2);
  [F,Fdot] = getFandFdot(obj,t,x,u);
%   display(sprintf('%f: %c, %c',t,max(svd(F)),max(abs(svd(Fdot)))))
  A = kron(eye(n-d),F);
  b = -reshape(Fdot*P + alpha_1*F*P,[],1);
  ortho_err = eye(n-d) - P'*P;
  
  %d/dt P'*P = 0 
  row_ind = size(A,1) + 1;
  A = [A;zeros(((n-d)^2 + (n-d))/2,n*(n-d))]; 
  b = [b;zeros(((n-d)^2 + (n-d))/2,1)];
  
  for i=1:n-d,
    A(row_ind:row_ind+i-1,1:n*i) = kron(eye(i),P(:,i)');
    A(row_ind:row_ind+i-1,(1:n) + n*(i-1)) = A(row_ind:row_ind+i-1,(1:n) + n*(i-1)) + P(:,1:i)';    
    b(row_ind:row_ind+i-1) = alpha_2*ortho_err(i,1:i);
    row_ind = row_ind + i;
  end
  
if cond(A) > 1e3  %results from F*P != 0
  warning('The A matrix in getPdot is poorly conditioned.  Implies that J is poorly conditioned OR F*P != 0')
  display(sprintf('%f %e',t, cond(A)));
%   keyboard
end
% display(sprintf('P:%f',t))
Pdot = reshape(pinv(A)*b,n,n-d);
% Pdot = reshape(A\b,n,n-d);
% [Q,R] = qr(A');
% Pdot = reshape(Q'*(R'\b),n,n-d);

% if max(max(abs(F*P))) > .1
%   keyboard
% end

%   display(sprintf('t: %f FPerr: %e',t,max(max(abs(F*P)))))
end

function PandSqrtSdotydot = PandSqrtSdynamics(t,PandSqrtS,p,dynamicsfn,xtraj,utraj,Q,R,alpha_1,alpha_2,dimP)
  % S_dot = -A'S - SA + SBinv(R)B'S - Q
  
  P_t = reshape(PandSqrtS(1:dimP),2*p.getNumPositions(),[]);  
  x_t = xtraj.eval(t);
  u_t = utraj.eval(t);
 
  Pdot_t = getPdot(p,t,x_t,u_t,P_t,alpha_1,alpha_2);

  [xd,dxd] = dynamicsfn(t,x_t,u_t);
  A = dxd(:,2:2*p.getNumPositions()+1);
  B = dxd(:,2+2*p.getNumPositions():end);

  % A_t = P_t'*A*P_t + P_t'*Pdot_t;  %was this
  A_t = P_t'*A*P_t + Pdot_t'*P_t;
  B_t = P_t'*B;

  sqrtS = reshape(PandSqrtS(dimP+1:end),size(A_t,1),[]);

%   if cond(sqrtS) > 1e4
%     keyboard
%   end
  

  sqrtSdot = -A_t'*sqrtS + .5*sqrtS*sqrtS'*B_t*inv(R)*B_t'*sqrtS - .5*P_t'*Q*P_t*inv(sqrtS)';

  PandSqrtSdotydot = [Pdot_t(:);sqrtSdot(:)];
%   display(sprintf('t: %f xd: %e',t,norm(xd)))
  % display(sprintf('S: %f',t))  
%   display(sprintf('%f: %c',t,cond(sqrtS)))
%   display(sprintf('%f: %c, %c',t,max(abs(eig(A))),max(abs(eig(A_t)))))
%   display(sprintf('%f: %c, %c',t,max(svd(Pdot_t)),max(abs(eig(A_t)))))
end

function Pdot = Pdynamics(obj,t,xtraj,utraj,P,alpha_1,alpha_2)
  x = xtraj.eval(t);
  u = utraj.eval(t);
  P = reshape(P,2*obj.getNumPositions(),[]);
  Pdot = getPdot(obj,t,x,u,P,alpha_1,alpha_2);
  Pdot = Pdot(:);
%   t
end

function Sdot = Sdynamics(t,S,p,dynamicsfn,xtraj,utraj,Ptraj,Q,R,alpha_1,alpha_2)
  % S_dot = -A'S - SA + SBinv(R)B'S - Q
  x_t = xtraj.eval(t);
  u_t = utraj.eval(t);
  P_t = Ptraj.eval(t);
  Pdot_t = getPdot(p,t,x_t,u_t,P_t,alpha_1,alpha_2);

  [xd,dxd] = dynamicsfn(t,x_t,u_t);
  A = dxd(:,2:2*p.getNumPositions()+1);
  B = dxd(:,2+2*p.getNumPositions():end);

  % A_t = P_t'*A*P_t + P_t'*Pdot_t;  %was this
  A_t = P_t'*A*P_t + Pdot_t'*P_t;
  B_t = P_t'*B;

  S = reshape(S,size(A_t,1),[]);

  Sdot = -A_t'*S - S*A_t + S*B_t*inv(R)*B_t'*S - P_t'*Q*P_t;

  Sdot = Sdot(:);
%   display(sprintf('S: %f',t))
end

function sqrtSdot = sqrtSdynamics(t,sqrtS,p,dynamicsfn,xtraj,utraj,Ptraj,Q,R,alpha_1,alpha_2)
% S_dot = -A'S - SA + SBinv(R)B'S - Q
x_t = xtraj.eval(t);
u_t = utraj.eval(t);
P_t = Ptraj.eval(t);
Pdot_t = getPdot(p,t,x_t,u_t,P_t,alpha_1,alpha_2);

[xd,dxd] = dynamicsfn(t,x_t,u_t);
A = dxd(:,2:2*p.getNumPositions()+1);
B = dxd(:,2+2*p.getNumPositions():end);
  
% A_t = P_t'*A*P_t + P_t'*Pdot_t;  %was this
A_t = P_t'*A*P_t + Pdot_t'*P_t;
B_t = P_t'*B;
  
sqrtS = reshape(sqrtS,size(A_t,1),[]);

sqrtSdot = -A_t'*sqrtS + .5*sqrtS*sqrtS'*B_t*inv(R)*B_t'*sqrtS - .5*P_t'*Q*P_t*inv(sqrtS)';

sqrtSdot = sqrtSdot(:);
% display(sprintf('S: %f',t))
end

