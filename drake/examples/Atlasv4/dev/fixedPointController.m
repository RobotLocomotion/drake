function [u,K,c,S] = fixedPointController(p,q)

% Find nominal u and LQR controller
qd = q*0;
[phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = contactConstraints(p,q);
[H,C,B] = p.manipulatorDynamics(q,qd);

nU = p.getNumInputs;
nC = length(phi);
nD = length(D);
%% Find the fixed point with a QP
nVars = nU+nC*(1+nD);
H_qp = eye(nVars);
H_qp(1:nU,1:nU) = 100*eye(nU);

J_T = n';
for i=1:nD,
  J_T = [J_T D{i}'];
end
A_eq = [B J_T];
b_eq = C;

A_in = [zeros(nC,nU) -.25*eye(nC) repmat(eye(nC),1,nD)];
b_in = zeros(nC,1);

lb = [-inf(nU,1);0*ones(nC,1);zeros(nC*nD,1)];
ub = inf(nVars,1);
[z,~,flag] = quadprog(H_qp,zeros(nVars,1),A_in,b_in,A_eq,b_eq,lb,ub);
u = z(1:nU)
flag

% %% Constrained LQR
% plant = p;
% 
% 
% for i=1:length(phi),
%   xA_i = xA(:,i);
%   xB_i = xB(:,i);
%   if idxA(i) == 1,
%     position_fun = drakeFunction.kinematic.RelativePosition(plant,idxB(i),idxA(i),xB_i);
%     position_constraint = DrakeFunctionConstraint(xA_i,xA_i,position_fun);
%   else
%     position_fun = drakeFunction.kinematic.RelativePosition(plant,idxA(i),idxB(i),xA_i);
%     position_constraint = DrakeFunctionConstraint(xB_i,xB_i,position_fun);
%   end
%   position_constraint.grad_level = 2;
%   
%   plant = plant.addPositionEqualityConstraint(position_constraint);
% end
% 
% [xdot,dxdot] = constrainedDynamics(plant,0,[q;qd],u)

%%
cplant = p;
assert(length(D) == 4) %if another friction basis is used, this code needs updating

if idxB(1) == 1,
  tmp = idxB;
  idxB = idxA;
  idxA = tmp;
  tmp = xB;
  xB = xA;
  xA = tmp;
end

J = [n(1,:); D{1}(1,:); D{2}(1,:)];

position_fun = drakeFunction.kinematic.RelativePosition(cplant,idxB(1),idxA(1),xB(:,1));
position_constraint = DrakeFunctionConstraint(xA(:,1),xA(:,1),position_fun);
position_constraint.grad_level = 2;
cplant = cplant.addPositionEqualityConstraint(position_constraint);

for j=2:length(phi)
  if cond([J;n(j,:)]) < 1e2
    coord_inds = 3;
    J = [J; n(j,:)];
  else
    coord_inds = [];
  end
  
  i = j;
  
  if cond([J;D{2}(j,:)]) < 1e2
    coord_inds = [2;coord_inds];
    J = [J;D{2}(j,:)];
  end
  
  if cond([J;D{1}(j,:)]) < 1e2
    coord_inds = [1;coord_inds];
    J = [J;D{1}(j,:)];
  end
  
  if ~isempty(coord_inds)
    posA = xA(coord_inds,i);
    position_fun = drakeFunction.kinematic.RelativePosition(cplant,idxB(i),idxA(i),xB(:,i),coord_inds);
    position_constraint = DrakeFunctionConstraint(posA,posA,position_fun);
    position_constraint.grad_level = 2;
    cplant = cplant.addPositionEqualityConstraint(position_constraint);
  end
end

[xdot,dxdot] = constrainedDynamics(cplant,0,[q;qd],u);

%%
[~,J,dJ,Jdotqd,dJdotqd] = cplant.positionConstraintslWithJdot(q,qd);
dJ = reshape(dJ,numel(J),[]);
Jdot = reshape(dJ * qd, size(J));
F = full([J zeros(size(J));Jdot J]);
P = null(F);
A_sys = P'*dxdot(:,2:p.getNumStates+1)*P;
B_sys = P'*dxdot(:,2+p.getNumStates:end);

Q = diag([100*ones(p.getNumPositions,1);10*ones(p.getNumVelocities,1)]);
Q_sys = P'*Q*P;
R_sys = .01*eye(getNumInputs(p));
[K,S] = lqr(A_sys,B_sys,Q_sys,R_sys);

K = K*P';

c = AffineSystem([],[],[],[],[],[],[],-K,K*[q;qd] + u);
c = c.setOutputFrame(p.getInputFrame);
c = c.setInputFrame(p.getOutputFrame);