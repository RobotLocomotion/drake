function [A,dA] = getCMMdA(model,kinsol)
% returns the centroidal momentum matrix as described in Orin & Goswami 2008
%
% h = A*qd, where h(4:6) is the total linear momentum and h(1:3) is the 
% total angular momentum in the centroid frame (world fram translated to COM).

if ~isstruct(kinsol)
  % treat input as getCMM(model,q)
  kinsol = doKinematics(model,kinsol,false,false);
end

if (kinsol.mex)
  error('Drake:RigidBodyManipulator:getCMMdA:noMex', ...
        'getCMMdA is not implemented in mex yet');
else
  q = kinsol.q;
  nq = getNumDOF(model);
  m = model.featherstone;

  A = zeros(6,nq) + 0*q(1);
  Phi = cell(m.NB,1); 
  Xup = cell(m.NB,1); % spatial transforms from parent to child 
  Xworld = cell(m.NB,1); % spatial transforms from world to each body
  Ic = cell(m.NB,1); % body spatial inertias

  compute_first_derivative = (nargout > 1);

  if compute_first_derivative
    dA = zeros(6,nq,nq) + 0*q(1);
    dXup_dq = cell(m.NB,1);
    dXworld_dq = cell(m.NB,1);
    [com, Jcom] = getCOM(model,kinsol);
    dXtr = dXtrans(-com); 
    % prob not efficient
    dXcom_dq = -dXtr*Jcom;
    dIc = cell(m.NB,1); % derivative of body spatial inertias
    dIc_dq = cell(m.NB,1); % derivative of body spatial inertias
  else
    com = getCOM(model,kinsol);
  end
  Xcom = Xtrans(-com); % spatial transform from centroid to world
  
  for i = 1:m.NB
    Ic{i} = m.I{i};
    dIc{i} = zeros(6,6) + 0*q(1);
    dIc_dq{i} = zeros(36,nq) + 0*q(1);
  end
  
  for i = m.NB:-1:1
    n = m.dofnum(i);
    [Xi,phi] = jcalc(m.pitch(i), q(n));
    Xup{i} = Xi * m.Xtree{i} + 0*q(n);
  
    if compute_first_derivative
      dXi_dq = zeros(36,nq) + 0*q(1);
      dXi_dq(:,n) = reshape(djcalc(m.pitch(i), q(n)),36,1);
      %dXup_dq{i} = zeros(36,nq);
      %dXup_dq{i}(:,n) = reshape(dXidq * m.Xtree{i},36,1);
      dXup_dq{i} = matGradMultMat(Xi,m.Xtree{i},dXi_dq,zeros(36,nq));
    end
    
    Phi{i} = phi;
    if m.parent(i) > 0
      Ic{m.parent(i)} = Ic{m.parent(i)} + Xup{i}'*Ic{i}*Xup{i};

      if compute_first_derivative
        dIc_dq{m.parent(i)} = dIc_dq{m.parent(i)} + matGradMultMat(Xup{i}',Ic{i}*Xup{i},transposeGrad(dXup_dq{i},[6,6]),matGradMultMat(Ic{i},Xup{i},dIc_dq{i},dXup_dq{i}));
      end
    end
  end
  
  for i = 1:m.NB
    n = m.dofnum(i);
    if m.parent(i) > 0
      Xworld{i} = Xup{i} * Xworld{m.parent(i)};
      if compute_first_derivative
        dXworld_dq{i} = matGradMultMat(Xup{i},Xworld{m.parent(i)},dXup_dq{i},dXworld_dq{m.parent(i)});
      end      
    else
      Xworld{i} = Xup{i};
      if compute_first_derivative
        dXworld_dq{i} = dXup_dq{i};
      end  
    end
    Xg = Xworld{i} * Xcom; % spatial transform from centroid to body
    A(:,n) = Xg'*Ic{i}*Phi{i};
    
    if compute_first_derivative
      dXg_dq = matGradMultMat(Xworld{i},Xcom,dXworld_dq{i},dXcom_dq); 
      dA(:,n,:) = matGradMultMat(Xg',Ic{i}*Phi{i},transposeGrad(dXg_dq,size(Xg)),matGradMult(dIc_dq{i},Phi{i}));
    end
  end
  %A = Xcom;
  if compute_first_derivative
    dA = reshape(dA,6*nq,nq);
    %dA = dXcom_dq;
  end
end
end
