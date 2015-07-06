function [n, D, dn, dD] = contactConstraintDerivatives(obj, normal, kinsol, idxA, idxB, xA, xB, d)

compute_second_derivative = nargout > 3;

nq = obj.getNumPositions;  
nk = size(d,2);
nC = numel(idxA);
J = zeros(3*nC,nq);

if compute_second_derivative,
  dJ = zeros(3*nC,nq*nq);
end

body_inds = unique([idxA(idxA>1);idxB(idxB>1)]);

% Cache the results of a kron and repmat, since this was taking a ton of
% time
tmp_vec = repmat([-2;-1;0],nC,1);
tmp_kron = kron(eye(nC),3*ones(3,1));

% loop over the bodies with contact points, instead of number of contacts
% reduces the calls to forwardKin from n^2 to n
for i=1:length(body_inds),
  % The contact indices related to this body, on idxA and idxB
  cindA = find(idxA == body_inds(i));
  cindB = find(idxB == body_inds(i));
  % Vectorized calculation of Jacobian indices related to this contact
  % for contact i, this is (1:3) + (i-1)*3
  if ~isempty(cindA)
    JindA = tmp_kron(1:3*length(cindA),1:length(cindA))*cindA + tmp_vec(1:3*length(cindA));
  else
    JindA = [];
  end
  if ~isempty(cindB)
    JindB = tmp_kron(1:3*length(cindB),1:length(cindB))*cindB + tmp_vec(1:3*length(cindB));
  else
    JindB = [];
  end
  if compute_second_derivative
    [~,J_tmp,dJ_tmp] = obj.forwardKin(kinsol,body_inds(i), [xA(:,cindA) xB(:,cindB)]);
    dJ(JindA,:) = dJ(JindA,:) + dJ_tmp(1:3*length(cindA),:);
    dJ(JindB,:) = dJ(JindB,:) - dJ_tmp(3*length(cindA)+1:end,:);
  else
    [~,J_tmp] = obj.forwardKin(kinsol, body_inds(i),  [xA(:,cindA) xB(:,cindB)]);
  end
  J(JindA,:) = J(JindA,:) + J_tmp(1:3*length(cindA),:);
  J(JindB,:) = J(JindB,:) - J_tmp(3*length(cindA)+1:end,:);
end
indmat = repmat(1:nC,3,1);
n = sparse(indmat,1:3*nC,normal(:))*J;

D = cell(1,nk);
dD = cell(1,nk);
for k=1:nk,
  D{k} = sparse(indmat,1:3*nC,d{k}(:))*J;
  if compute_second_derivative
    % note: this temporarily assumes that the normal does not change with contact_pos
    dD{k} = reshape(sparse(indmat,1:3*nC,d{k}(:))*dJ,numel(n),nq);
  end
end

for k=(nk+1):2*nk
  D{k} = -D{k-nk};
  if compute_second_derivative
    dD{k} = -dD{k-nk};
  end
end

if compute_second_derivative
  dn = reshape(sparse(indmat,1:3*nC,normal(:))*dJ,numel(n),nq);
end

end

