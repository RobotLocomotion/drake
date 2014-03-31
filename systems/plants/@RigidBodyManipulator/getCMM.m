function [A,Adot] = getCMM(model,kinsol,qd)
% returns the centroidal momentum matrix as described in Orin & Goswami 2008
%
% h = A*qd, where h(4:6) is the total linear momentum and h(1:3) is the 
% total angular momentum in the centroid frame (world fram translated to COM).

if nargout > 1 && nargin < 3
  error('RigidBodyManipulator:getCMM:NotEnoughArguments',...
        'If you ask for Adot, you must provide qdot.');
end

if ~isstruct(kinsol)
  % treat input as getCMM(model,q)
  kinsol = doKinematics(model,kinsol,false);
end

if (kinsol.mex)
  if (model.mex_model_ptr==0)
    error('RigidBodyManipulator:getCMM:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if nargout > 1
    [A,Adot] = getCMMmex(model.mex_model_ptr,kinsol.q,qd);
	else
		A = getCMMmex(model.mex_model_ptr,kinsol.q);
	end
else
  q = kinsol.q;
  nq = getNumDOF(model);
  m = model.featherstone;

  A = zeros(6,nq) + 0*q(1);
  Phi = cell(m.NB,1); 
  Xup = cell(m.NB,1); % spatial transforms from parent to child 
  Xworld = cell(m.NB,1); % spatial transforms from world to each body
  Ic = cell(m.NB,1); % body spatial inertias

  if nargout > 1
    Adot = zeros(6,nq) + 0*q(1);
    dXup = cell(m.NB,1); % dXup_dq * qd
    dXworld = cell(m.NB,1); % dXworld_dq * qd
    [com, Jcom] = getCOM(model,kinsol);
    xdot_com = Jcom*qd;
    dXtr = dXtrans(-com); 
    % prob not efficient
    dXcom = -reshape(dXtr(:,1),[6 6])*xdot_com(1) - ...
        reshape(dXtr(:,2),[6 6])*xdot_com(2) - ...
        reshape(dXtr(:,3),[6 6])*xdot_com(3);
    dIc = cell(m.NB,1); % derivative of body spatial inertias
  else
    com = getCOM(model,kinsol);
  end
  Xcom = Xtrans(-com); % spatial transform from centroid to world
  
  for i = 1:m.NB
    Ic{i} = m.I{i};
    dIc{i} = zeros(6,6);
  end
  
  for i = m.NB:-1:1
    n = m.dofnum(i);
    [Xi,phi] = jcalc(m.pitch(i), q(n));
    Xup{i} = Xi * m.Xtree{i} + 0*q(n);
  
    if nargout > 1
      dXidq = djcalc(m.pitch(i), q(n));
      dXup{i} = dXidq * m.Xtree{i} * qd(n);
    end
    
    Phi{i} = phi;
    if m.parent(i) > 0
      Ic{m.parent(i)} = Ic{m.parent(i)} + Xup{i}'*Ic{i}*Xup{i};

      if nargout > 1
        dIc{m.parent(i)} = dIc{m.parent(i)} + (dXup{i}'*Ic{i} + Xup{i}'*dIc{i})*Xup{i} + Xup{i}'*Ic{i}*dXup{i};
      end
    end
  end
  
  for i = 1:m.NB
    n = m.dofnum(i);
    if m.parent(i) > 0
      Xworld{i} = Xup{i} * Xworld{m.parent(i)};
      if nargout > 1
        dXworld{i} = dXup{i}*Xworld{m.parent(i)} + Xup{i}*dXworld{m.parent(i)};
      end      
    else
      Xworld{i} = Xup{i};
      if nargout > 1
        dXworld{i} = dXup{i};
      end  
    end
    Xg = Xworld{i} * Xcom; % spatial transform from centroid to body
    A(:,n) = Xg'*Ic{i}*Phi{i};
    
    if nargout > 1
      dXg = dXworld{i} * Xcom + Xworld{i} * dXcom; 
      Adot(:,n) = dXg'*Ic{i}*Phi{i} + Xg'*dIc{i}*Phi{i};
    end
  end
end
end