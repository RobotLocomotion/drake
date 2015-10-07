function [c,Ktraj,Straj,Ptraj,Btraj,Straj_full,Ftraj,xtraj,utraj] = hybriddircolconstrainedtvlqr(obj,xtraj,utraj,contact_seq,Q,R,Qf,options)
%HYBRIDCONSTRAINEDTVLQR
% TVLQR for a hybrid model with constraints
% @input obj The plant object
% @input xtraj Nominal state trajectory
% @input utraj Nominal input trajectory
% @input ltraj Nominal constraint force trajectory (for extracting modes)
%        The indices of the constraints which are ordered
%        [joint_limits;contact_points] where each contact point
%        is 2x1 (tangential, normal)
% @input Q
% @input R
% @input Qf
% @input options.tspan time span, defaults to using the xtraj tspan
% Performs time varying lqr on a reduced order model
% @input options.force_threshold Threshold to determine if the force
%                                is active, default 1e-3
% @input options.periodic Attemps to find a periodic solution (default 0)
% @input options.periodic_jump Jump condition where x(0) = F*x(end)
% @return c Cell of controller objects, for each hybrid mode
% @return Ktraj Cell of control gains
% @return Straj Cell of quadratic cost terms
% @return Ptraj Cell of Orthonormal bases of controllable states
% @return Btraj Cell of B(t), where xdot = A(t)x + B(t)u
% @return tvec M+1 (for M modes) vector of times in the mode sequence
%              Where the system is in mode i for t in (tvec(i),tvec(i+1))
if nargin < 8
  options = struct();
end



if ~isfield(options,'periodic')
  options.periodic = 0;
end

if ~isfield(options,'periodic_jump')
  options.periodic_jump = eye(obj.getNumStates);
end

if ~isfield(options,'use_joint_limits')
  options.use_joint_limits = false;
end



N = length(contact_seq);
constrained_plants = cell(N,1);
for i=1:N,
  cplant = obj;    
  x0 = xtraj{i}.eval(xtraj{i}.tspan(1));
  if ~isempty(contact_seq{i})    
    [phi,normal,d,xA,xB,idxA,idxB,mu,n,D] = contactConstraints(obj,x0(1:obj.getNumPositions));
    
    if idxB(1) == 1,
      tmp = idxB;
      idxB = idxA;
      idxA = tmp;
      tmp = xB;
      xB = xA;
      xA = tmp;
    end
    
    J = [n(contact_seq{i}(1),:); D{1}(contact_seq{i}(1),:)];
    
    %add constraint
    ind = contact_seq{i}(1);
    position_fun = drakeFunction.kinematic.RelativePosition(cplant,idxB(ind),idxA(ind),obj.T_2D_to_3D'*xB(:,ind));
    position_constraint = DrakeFunctionConstraint(obj.T_2D_to_3D'*xA(:,ind),obj.T_2D_to_3D'*xA(:,ind),position_fun);
    position_constraint.grad_level = 2;
    cplant = cplant.addPositionEqualityConstraint(position_constraint);
    
    for j=2:length(contact_seq{i})
      J = [J; n(contact_seq{i}(j),:)];
      ind = contact_seq{i}(j);
            
      if cond([J;D{1}(contact_seq{i}(j),:)]) < 1e2
        posA = obj.T_2D_to_3D'*xA(:,ind);
        J = [J;D{1}(contact_seq{i}(j),:)];
        position_fun = drakeFunction.kinematic.RelativePosition(cplant,idxB(ind),idxA(ind),obj.T_2D_to_3D'*xB(:,ind));
        position_constraint = DrakeFunctionConstraint(posA,posA,position_fun);
        position_constraint.grad_level = 2;
        cplant = cplant.addPositionEqualityConstraint(position_constraint);
      else
        posA = obj.T_2D_to_3D(:,2)'*xA(:,ind);
        position_fun = drakeFunction.kinematic.RelativePosition(cplant,idxB(ind),idxA(ind),obj.T_2D_to_3D'*xB(:,ind),2);
        position_constraint = DrakeFunctionConstraint(posA,posA,position_fun);
        position_constraint.grad_level = 2;
        cplant = cplant.addPositionEqualityConstraint(position_constraint);
      end
    end
  end
  
  if i > 1
    if ~isempty(setdiff(contact_seq{i},contact_seq{i-1}))
      [xp,F_i] = jump(cplant,xtraj{i-1}.eval(xtraj{i-1}.tspan(2)));
    else
      F_i = eye(obj.getNumStates);
    end
    
    F{i} = F_i;
  end
  
  constrained_plants{i} = cplant;
end



display(sprintf('Identified %d modes, with timings:',N));
for i=1:N
  display(sprintf('t %f to %f',xtraj{i}.tspan(1),xtraj{i}.tspan(2)));
end

if options.periodic
  jmax = 3;
else
  jmax = 1;
end
  
Qfi = Qf;
for j = 1:jmax;
  for i = N:-1:1,  
    ts = xtraj{i}.tspan;   
    [c{i},Ktraj{i},Straj{i},Ptraj{i},Btraj{i},Ftraj{i},Straj_full{i}] = constrainedtvlqr(constrained_plants{i},xtraj{i},utraj{i},Q,R,Qfi,struct('tspan',ts));
    P0 = Ptraj{i}.eval(ts(1));
    S0 = Straj{i}.eval(ts(1));
    Qfi = pinv(P0*P0')*P0*S0*P0'*pinv(P0*P0')';  %extract least squares full rank solution
    
    if i > 1 || options.periodic
      if i~=1
        Qfi = F{i}'*Qfi*F{i};
      else
        Qfi = options.periodic_jump'*Qfi*options.periodic_jump;
      end
      Qfi = (Qfi+Qfi')/2 + 1e-6*eye(size(Qfi,1));
      min(eig(Qfi))
    end
  end
end
end

function [xp,dxp] = jump(obj,xm)
nq = obj.getNumPositions;
if obj.num_position_constraints > 0
  q=xm(1:nq);
  qd=xm((nq+1):end);
  [phi,J,dJ] = obj.positionConstraints(q);
  [H,~,~,dH] = manipulatorDynamics(obj,q,qd);
  Hinv = inv(H);
  qdp = qd-Hinv*J'*inv(J*Hinv*J')*J*qd;
  xp = [q;qdp];
  
  if nargout > 1
    dJ = reshape(dJ,prod(size(J)),[]);
    dJ_transpose = transposeGrad(dJ,size(J));
    dH = dH(:,1:nq);
    dHinv = invMatGrad(H,dH);
    % gradient of J*Hinv*J'
    R = J*Hinv*J';
    dR = matGradMultMat(J*Hinv,J',matGradMultMat(J,Hinv,dJ,dHinv),dJ_transpose);
    % J'*inv(J*Hinv*J')
    Y = J'*pinv(R);
    dY = matGradMultMat(J',inv(R),dJ_transpose,invMatGrad(R,dR));
    % J'*inv(J*Hinv*J')*J
    Z = Y*J;
    dZ = matGradMultMat(Y,J,dY,dJ);
    % inv(H)*J'*inv(J*Hinv*J')*J
    ZZ = Hinv*Z;
    dZZ = matGradMultMat(Hinv,Z,dHinv,dZ);
    
    dvpdq = -matGradMult(dZZ,qd);
    dvpdv = eye(nq) - Hinv*J'*inv(J*Hinv*J')*J;
    dxp = [eye(nq) zeros(nq);
      dvpdq dvpdv];
  end
else
  xp = xm;
  dxp = eye(length(xm));
end
end
