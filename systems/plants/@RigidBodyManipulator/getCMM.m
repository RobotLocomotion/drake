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
% 		disp('Adot');
		dertic = tic;
    [A,Adot] = getCMMmex(model.mex_model_ptr,kinsol.q,qd);
		toc(dertic);
	else
% 		disp('A');
		tic;
		A = getCMMmex(model.mex_model_ptr,kinsol.q);
		toc;
	end
  
else
  q = kinsol.q;
  nq = getNumDOF(model);
  m = model.featherstone;

  P = eye(6*m.NB,6*m.NB) + 0*q(1); % spatial incidence matrix (eq 9)
  Phi = zeros(6*m.NB,nq) + 0*q(1); % joint axis matrix (eq 12)
  I = zeros(6*m.NB,6*m.NB) + 0*q(1); % system inertia matrix (eq 18)

  Xg = zeros(6*m.NB,6) + 0*q(1); % spatial centroidal projection matrix (eq 25)
  Xup = cell(m.NB,1); % spatial transforms from parent to child 
  Xworld = cell(m.NB,1); % spatial transforms from world to each body

  if nargout > 1
    dXup = cell(m.NB,1); % dXup_dq * qd
    dP = zeros(6*m.NB,6*m.NB); % dP_dq * qd 
    dXworld = cell(m.NB,1); % dXworld_dq * qd
    dXg = zeros(6*m.NB,6); 
    [com, Jcom] = getCOM(model,kinsol);
    xdot_com = Jcom*qd;
    dXtr = dXtrans(-com); 
    % prob not efficient
    dXcom = -reshape(dXtr(:,1),[6 6])*xdot_com(1) - ...
        reshape(dXtr(:,2),[6 6])*xdot_com(2) - ...
        reshape(dXtr(:,3),[6 6])*xdot_com(3);
  else
    com = getCOM(model,kinsol);
  end

  Xcom = Xtrans(-com); % spatial transform from centroid to world

  for i = 1:m.NB
    n = m.dofnum(i);
    [Xi,s] = jcalc(m.pitch(i), q(n));
    Xup{i} = Xi * m.Xtree{i} + 0*q(n);

    if nargout > 1
      dXidq = djcalc(m.pitch(i), q(n));
      dXup{i} = dXidq * m.Xtree{i} * qd(n);
    end

    if m.parent(i) > 0
      P((i-1)*6+(1:6),(m.parent(i)-1)*6+(1:6)) = -Xup{i};
      Xworld{i} = Xup{i} * Xworld{m.parent(i)};
      if nargout > 1
        dP((i-1)*6+(1:6),(m.parent(i)-1)*6+(1:6)) = -dXup{i};
        dXworld{i} = dXup{i}*Xworld{m.parent(i)} + Xup{i}*dXworld{m.parent(i)};
      end
    else
      Xworld{i} = Xup{i};
      if nargout > 1
        dXworld{i} = dXup{i};
      end  
    end

    Phi((i-1)*6+(1:6),n) = s;
    I((i-1)*6+(1:6),(i-1)*6+(1:6)) = m.I{i};

    Xg((i-1)*6+(1:6),:) = Xworld{i} * Xcom; % spatial transform from centroid to body
    if nargout > 1
      dXg((i-1)*6+(1:6),:) = dXworld{i} * Xcom + Xworld{i} * dXcom; 
    end
  end

  if nargout > 1
    Pinv = inv(P);
    J = Pinv*Phi;
    Jdot = -Pinv * dP * Pinv * Phi;
    Adot = Xg'*I*Jdot + dXg'*I*J;
  else
    J = P\Phi;
  end

  % A = I*J; % system momentum matrix (eq 20)
  A = Xg'*I*J; % centroidal momentum matrix (eq 27)
end