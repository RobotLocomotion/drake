function [c,Ktraj,Straj,Ptraj,Btraj,Ftraj,Straj_full] = constrainedtvlqr(obj,xtraj,utraj,Q,R,Qf,constraint_ind,options)
%CONSTRAINEDTVLQR
% TVLQR with constraints
% @input obj The plant object
% @input xtraj Nominal state trajectory
% @input utraj Nominal input trajectory
% @input Q 
% @input R
% @input Qf
% @input constraint_ind The indices of the constraints which are active
%        ordered [joint_limits;contact_points] where each contact point
%        is 2x1 (tangential, normal)
% @input options.tspan time span, defaults to using the xtraj tspan
% @input options.P0 Optionally, supply P0
% Performs time varying lqr on a reduced order model
% @return c The controller object
% @return Ktraj control gains
% @return Straj quadratic cost term
% @return Ptraj Orthonormal basis of controllable states
% @return Btraj B(t), where xdot = A(t)x + B(t)u

if nargin < 8
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
  alpha_1 = 200; % was 5000
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

if ~isfield(options,'use_joint_limits')
  options.use_joint_limits = false;
end

x0 = xtraj.eval(tspan(1));
u0 = utraj.eval(tspan(1));

F0 = getFandFdot(obj,0,x0,u0,constraint_ind,options);

% if isfield(options,'P0')
%   P0 = options.P0;
%   if any(abs(F0*P0) > 1e-2)
%     error('F0*P0 should be close to 0')
%   end  
% else
  P0 = null(F0);
% end

dynamicsFun = @(t,x,u) constrainedDynamics(obj,t,x,u,constraint_ind,options);


% % update utraj more finely
% t_vec = linspace(tspan(1),tspan(2),1e2);
% u_vec = zeros(size(R,1),length(t_vec));
% for i=1:length(t_vec),
%   x_i = xtraj.eval(t_vec(i));
%   u_i = utraj.eval(t_vec(i));
%   xdot_nom_i = xtraj.deriv(t_vec(i));
%   [xdot_i,A] = geval(dynamicsFun,t_vec(i),x_i,u_i,struct('grad_method','numerical'));
%   
%   u_vec(:,i) = u_i + A(:,2+length(x0):end)\(xdot_nom_i - xdot_i);  
%   i
% end
% utraj = PPTrajectory(foh(t_vec,u_vec));


Pfun = @(t,P) Pdynamics(obj,t,xtraj,utraj,P,constraint_ind,alpha_1,alpha_2,options);
[tout,yout] = ode45(Pfun,tspan,P0);

nQ = obj.num_q;                         %unconstrained position state size
nQC = length(constraint_ind);          %number of constrained positions

Ptraj = PPTrajectory(foh(tout,reshape(yout',2*nQ,2*nQ-2*nQC,[])));
Pf = Ptraj.eval(tspan(2));

Sf = Pf'*Qf*Pf;
if ~options.sqrtmethod
  Sfun = @(t,S) Sdynamics(t,S,obj,dynamicsFun,xtraj,utraj,Ptraj,Q,R,constraint_ind,alpha_1,alpha_2,options);
else
  Sf = Sf^(1/2);
  Sfun = @(t,S) sqrtSdynamics(t,S,obj,dynamicsFun,xtraj,utraj,Ptraj,Q,R,constraint_ind,alpha_1,alpha_2,options);
end
[tout,yout] = ode45(Sfun,tspan(end:-1:1),Sf);

S_data = reshape(yout',2*nQ - 2*nQC,2*nQ - 2*nQC,[]);
Straj = PPTrajectory(foh(tout(end:-1:1),S_data(:,:,end:-1:1)));

if options.sqrtmethod
  Straj = Straj*Straj';
end

Btraj = getBTrajectory(obj,Straj.getBreaks(),xtraj,utraj,constraint_ind,options);

Ktraj = inv(R)*(Ptraj'*Btraj)'*Straj*Ptraj';

c = AffineSystem([],[],[],[],[],[],[],-Ktraj,Ktraj*xtraj + utraj);
c = c.setInputFrame(obj.getOutputFrame);
c = c.setOutputFrame(obj.getInputFrame);

Straj_full = Ptraj*Straj*Ptraj';

tt = Straj.pp.breaks;
F_data = zeros(size(F0,1),size(F0,2),length(tt));
for i=1:length(tt),
  F_data(:,:,i) = getFandFdot(obj,tt(i),xtraj.eval(tt(i)),utraj.eval(tt(i)),constraint_ind,options);
end
Ftraj = PPTrajectory(foh(tt,F_data));
end

function B = getBTrajectory(p,ts,xtraj,utraj,constraint_ind,options)
  nX = length(xtraj); nU = length(utraj);
  B = zeros([nX,nU,length(ts)]);
  dynamicsFun = @(t,x,u) constrainedDynamics(p,t,x,u,constraint_ind,options);
  for i=1:length(ts)
    x0=xtraj.eval(ts(i)); u0 = utraj.eval(ts(i));    
    [xd,dxd] = geval(dynamicsFun,ts(i),x0,u0,struct('grad_method','numerical'));
    B(:,:,i) = dxd(:,nX+1+(1:nU));
  end
  B = PPTrajectory(spline(ts,B));
end


function [F,Fdot]=getFandFdot(obj,t,x,u,constraint_ind,options)
q = x(1:obj.num_q);
qd = x(obj.num_q+1:end);


if options.use_joint_limits
  [phi_limit,J_limit,dJ_limit] = obj.jointLimitConstraints(q);
  dJ_limit = reshape(dJ_limit,[],length(q));
else
  phi_limit = [];
  J_limit = zeros(0,length(q));
  dJ_limit = zeros(0,length(q)^2);
end


[phi_n,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.contactConstraints(q);

J = zeros(2*size(n,1),size(n,2));
J(1:2:end,:) = D{1};
J(2:2:end,:) = n;

J = [J_limit;J];
J = J(constraint_ind,:);

phi = zeros(2*size(n,1),1);
% phi(1:2:end,:) = phi_f;
phi(2:2:end,:) = phi_n;
phi = [phi_limit;phi];
phi = phi(constraint_ind);
if any(phi > 1e-2)
%   keyboard
%   warning('Constraint does not evaluate to near 0')
end

dJqd = zeros(2*size(n,1),size(n,2));
dJqd(1:2:end,:) = matGradMult(dD{1},qd);
dJqd(2:2:end,:) = matGradMult(dn,qd);

if size(J_limit,1) > 0
  dJqd = [matGradMult(dJ_limit,qd); dJqd];
end
dJqd = dJqd(constraint_ind,:);

F = full([J zeros(size(J));dJqd J]);

if nargout > 1
  % Fdot = -F A
  dynamicsFun = @(t,x,u) constrainedDynamics(obj,t,x,u,constraint_ind,options);
  [~,A] = geval(dynamicsFun,t,x,u,struct('grad_method','numerical'));
  
  Fdot = - F*A(:,2:1+length(x));
end

% if nargout > 1
%   xd = constrainedDynamics(obj,t,x,u,constraint_ind);
%   qdd = xd(obj.num_q+1:end);
%   
%   [~,ddJqd] = geval(@obj.getdJgqd,q,qd,struct('grad_method','numerical'));
%   
%   ddJqddt0 = zeros(2*size(n,1),size(n,2));
%   ddJqddt0(1:2:end,:) = matGradMult(dD{1},qdd);
%   ddJqddt0(2:2:end,:) = matGradMult(dn,qdd);
%   
%   ddJqddt = ddJqddt0 + matGradMult(ddJqd(:,1:obj.num_q),qd);
%   
%   %assuming dJ is zeros for joint limits here, this is could be a bad idea
%   ddJqddt = [zeros(size(J_limit));ddJqddt];
%   
%   ddJqddt = ddJqddt(constraint_ind,:);
%   
%   Fdot = full([dJqd zeros(size(J)); ddJqddt dJqd]);
% end
end


% function [Jdot,dJdot] = getJdot(obj,q,v,constraint_ind)
%   [phi,normal,d,xA,xB,idxA,idxB] = obj.contactConstraints(q);
%   % a hack for now
%   assert(max(idxA) == min(idxA) == 1)
%   kinsol = obj.doKinematics(q,true,true,qd);
%   if nargout > 1
%     
%   else
%     
%     Jdot = obj.forwardJacDot(kinsol,idxB,xB);
%   end
% end

function Pdot = getPdot(obj,t,x,u,P,constraint_ind,alpha_1,alpha_2,options)
n = length(x);
d = n - size(P,2);
[F,Fdot] = getFandFdot(obj,t,x,u,constraint_ind,options);
A = kron(eye(n-d),F);
b = -reshape(Fdot*P + alpha_1*F*P,[],1);
ortho_err = eye(n-d) - P'*P;
for i=1:n-d,
  for j=1:i,
    row = zeros(1,n*(n-d));
    row((1:n) + n*(i-1)) = P(:,j)';
    row((1:n) + n*(j-1)) = row((1:n) + n*(j-1)) + P(:,i)';
    A = [A;row];
    b = [b;alpha_2*ortho_err(i,j)];
  end
end
if cond(A) > 1e3  %results from F*P != 0
  warning('The A matrix in getPdot is poorly conditioned.  Implies that J is poorly conditioned OR F*P != 0')
  display(sprintf('%f %e',t, cond(A)));
%   keyboard
end
t
Pdot = reshape(pinv(A)*b,n,n-d);

if any(abs(Pdot) > 1e2)
  f = 1;
end
end

function Pdot = Pdynamics(obj,t,xtraj,utraj,P,constraint_ind,alpha_1,alpha_2,options)
x = xtraj.eval(t);
u = utraj.eval(t);
P = reshape(P,2*obj.num_q,[]);
Pdot = getPdot(obj,t,x,u,P,constraint_ind,alpha_1,alpha_2,options);
Pdot = Pdot(:);
% t
end

function Sdot = Sdynamics(t,S,p,dynamicsfn,xtraj,utraj,Ptraj,Q,R,constraint_ind,alpha_1,alpha_2,options)
% S_dot = -A'S - SA + SBinv(R)B'S - Q
x_t = xtraj.eval(t);
u_t = utraj.eval(t);
P_t = Ptraj.eval(t);
Pdot_t = getPdot(p,t,x_t,u_t,P_t,constraint_ind,alpha_1,alpha_2,options);

[xd,dxd] = geval(dynamicsfn,t,x_t,u_t,struct('grad_method','numerical'));
A = dxd(:,2:2*p.num_q+1);
B = dxd(:,2+2*p.num_q:end);
  
% A_t = P_t'*A*P_t + P_t'*Pdot_t;  %was this
A_t = P_t'*A*P_t + Pdot_t'*P_t;
B_t = P_t'*B;
  
S = reshape(S,size(A_t,1),[]);

Sdot = -A_t'*S - S*A_t + S*B_t*inv(R)*B_t'*S - P_t'*Q*P_t;

Sdot = Sdot(:);
% t
end

function sqrtSdot = sqrtSdynamics(t,sqrtS,p,dynamicsfn,xtraj,utraj,Ptraj,Q,R,constraint_ind,alpha_1,alpha_2,options)
% S_dot = -A'S - SA + SBinv(R)B'S - Q
x_t = xtraj.eval(t);
u_t = utraj.eval(t);
P_t = Ptraj.eval(t);
Pdot_t = getPdot(p,t,x_t,u_t,P_t,constraint_ind,alpha_1,alpha_2,options);

[xd,dxd] = geval(dynamicsfn,t,x_t,u_t,struct('grad_method','numerical'));
A = dxd(:,2:2*p.num_q+1);
B = dxd(:,2+2*p.num_q:end);
  
% A_t = P_t'*A*P_t + P_t'*Pdot_t;  %was this
A_t = P_t'*A*P_t + Pdot_t'*P_t;
B_t = P_t'*B;
  
sqrtS = reshape(sqrtS,size(A_t,1),[]);

sqrtSdot = -A_t'*sqrtS + .5*sqrtS*sqrtS'*B_t*inv(R)*B_t'*sqrtS - .5*P_t'*Q*P_t*inv(sqrtS)';

sqrtSdot = sqrtSdot(:);
% t
end

function xdot = constrainedDynamics(obj,t,x,u,constraint_ind,options)
q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
[H,C,B] = manipulatorDynamics(obj,q,qd);
Hinv = inv(H);
if (size(constraint_ind) > 0)  
  if options.use_joint_limits
    [~,J_limit,dJ_limit] = obj.jointLimitConstraints(q);
  else
    J_limit = zeros(0,length(q));
    dJ_limit = zeros(0,length(q)^2);
  end

  [phi_n,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.contactConstraints(q);
  
  phi = zeros(2*length(phi_n),1);
%   phi(1:2:end) = phi_f;
  phi(2:2:end) = phi_n;
  
  J = zeros(length(phi), obj.num_q);
  J(1:2:end,:) = D{1};
  J(2:2:end,:) = n;
  
  dJ = zeros(length(phi), obj.num_q^2);
  dJ(2:2:end,:) = reshape(dn,length(phi_n),[]);
  dJ(1:2:end,:) = reshape(dD{1},length(phi_n),[]);
  
  J_full = [J_limit;J];
  dJ_full = [dJ_limit;dJ];
%   phi_full = [phi; phi_limit];
  
%   phi_sub = phi_full(constraint_ind);
  Jdotqd = dJ_full(constraint_ind,:)*reshape(qd*qd',obj.num_q^2,1);
  J_sub = J_full(constraint_ind,:);
  
  if isempty(u)
    constraint_force = -J_sub'*(pinv(J_sub*Hinv*J_sub')*(J_sub*Hinv*(-C) + Jdotqd));
  else
    constraint_force = -J_sub'*(pinv(J_sub*Hinv*J_sub')*(J_sub*Hinv*(B*u-C) + Jdotqd));
  end
else
  constraint_force = 0;
end
% constraint_force = 0;
if isempty(u)
  qdd = Hinv*(constraint_force - C);
else
  qdd = Hinv*(B*u + constraint_force - C);
end
xdot = [qd;qdd];
end