function A = getCMM(model,kinsol)
% returns the centroidal momentum matrix as described in Orin & Goswami 2008

if ~isstruct(kinsol)  
  kinsol = doKinematics(model,kinsol,false);
end

q = kinsol.q;
nq = getNumDOF(model);
m = model.featherstone;

P = eye(6*m.NB,6*m.NB); % spatial incidence matrix (eq 9)
Phi = zeros(6*m.NB,nq); % joint axis matrix (eq 12)
I = zeros(6*m.NB,6*m.NB); % system inertia matrix (eq 18)

Xg = zeros(6*m.NB,6); % spatial centroidal projection matrix (eq 25)
Xup = cell(m.NB,1); % spatial transforms from parent to child 
Xworld = cell(m.NB,1); % spatial transforms from world to each body

xcom = getCOM(model,kinsol);
Xcom = Xtrans(-xcom); % spatial transform from centroid to world

for i = 1:m.NB
  n = m.dofnum(i);
  [Xi,s] = jcalc(m.pitch(i), q(n));
  Xup{i} = Xi * m.Xtree{i};
  if m.parent(i) > 0
    P((i-1)*6+(1:6),(m.parent(i)-1)*6+(1:6)) = -Xup{i};
    Xworld{i} = Xup{i} * Xworld{m.parent(i)};
  else
    Xworld{i} = Xup{i};
  end
  
  Phi((i-1)*6+(1:6),n) = s;
  I((i-1)*6+(1:6),(i-1)*6+(1:6)) = m.I{i};

  Xg((i-1)*6+(1:6),:) = Xworld{i} * Xcom; % spatial transform from centroid to body
end

J = P\Phi;
% A = I*J; % system momentum matrix (eq 20)
A = Xg'*I*J; % centroidal momentum matrix (eq 27)

end