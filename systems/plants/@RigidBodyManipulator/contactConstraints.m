function [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = contactConstraints(obj,kinsol,allow_multiple_contacts,active_collision_options)
% function [phi,xA,xB,idxA,idxB,n,D,mu,dn,dD] = contactConstraints(obj,kinsol,body_idx)
% Compute the contact constraints for a manipulator, and relevent bases.
% The contact frame always points from body B to body A.
%
% @param obj
% @param kinsol
% @param allow_multiple_contacts Allow multiple contacts per body pair.
%      Optional, defaults to false.
% @param active_collision_options A optional struct to determine which
%    bodies and collision groups are checked. See collisionDetect.
% @retval phi (m x 1) Vector of gap function values (typically contact distance), for m possible contacts
% @retval normal (3 x m) Contact normal vector in world coordinates, points from B to A
% @retval d {k} (3 x m) Contact friction basis vectors in world coordinates, points from B to A
% @retval xA (3 x m) The closest point on body A to contact with body B, relative to body A origin and in body A frame
% @retval xB (3 x m) The closest point on body B to contact with body A, relative to body B origin and in body B frame
% @retval idxA (m x 1) The index of body A. 0 is the special case for the environment/terrain
% @retval idxB (m x 1) The index of body B. 0 is the special case for the environment/terrain
% @retval mu (m x 1) Coefficients of friction
% @retval n (m x n) normal vector in joint coordinates, state vector length n
% @retval D {k}(m x n) friction cone basis in joint coordinates, for k directions
% @retval dn (mn x n) dn/dq derivative
% @retval dD {k}(mn x n) dD/dq derivative

compute_first_derivative = nargout > 8;
compute_second_derivative = nargout > 10;

if nargin<3,
  allow_multiple_contacts = false;
end

if nargin<4,
  active_collision_options = struct();
end

if ~isstruct(kinsol)  
  % treat input as contactPositions(obj,q)
  kinsol = doKinematics(obj,kinsol,compute_second_derivative);
end

[phi,normal,xA,xB,idxA,idxB] = collisionDetect(obj,kinsol,allow_multiple_contacts,active_collision_options);
idxA = idxA';
idxB = idxB';
nC = numel(phi);

% If there are no potential collisions, return empty
if nC == 0
  d = [];
  mu = [];
  n = [];
  D = [];
  dn = [];
  dD = [];
  return;
end

% For now, all coefficients of friction are 1
mu = ones(nC,1);

% while surfaceTangentsmex is not explicitly dependent on the the mex_model_ptr, 
% it may be sufficient to check for the presence of Eigen.
% This is faster than checking for the presence of files.

if(obj.mex_model_ptr ~= 0) 
  d = surfaceTangentsmex(normal);
else
  d = obj.surfaceTangents(normal);  
end

if compute_first_derivative
  nq = obj.getNumPositions;  
  nk = size(d,2);
  
  J = zeros(3*nC,nq)*kinsol.q(1);
  if compute_second_derivative,
    dJ = zeros(3*nC,nq*nq)*kinsol.q(1);;
  end
  
%   assert(isequal(idxA,sort(idxA)))
  
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
    if compute_second_derivative,
      [~,J_tmp,dJ_tmp] = obj.forwardKin(kinsol,body_inds(i),[xA(:,cindA) xB(:,cindB)]);
      dJ(JindA,:) = dJ(JindA,:) + dJ_tmp(1:3*length(cindA),:);
      dJ(JindB,:) = dJ(JindB,:) - dJ_tmp(3*length(cindA)+1:end,:);
    else
      [~,J_tmp] = obj.forwardKin(kinsol,body_inds(i),[xA(:,cindA) xB(:,cindB)]);
    end
    J(JindA,:) = J(JindA,:) + J_tmp(1:3*length(cindA),:);
    J(JindB,:) = J(JindB,:) - J_tmp(3*length(cindA)+1:end,:);
  end
  
%   for i=1:nC,
% %     imax = find(idxA == idxA(i),1,'last');
%     I = (1:3) + (i-1)*3;
% %     I = (i-1)*3 + 1: 3*imax;
%     
%     % For each of the two bodies, if it is a real body (i.e. idx != 0),
%     % then add compute the relevent jacobians in joint coordinates
%     if idxA(i) ~= 1,
%       if compute_second_derivative,
%         [~,J(I,:),dJ(I,:)] = obj.forwardKin(kinsol,idxA(i),xA(:,i));
%       else
%         [~,J(I,:)] = obj.forwardKin(kinsol,idxA(i),xA(:,i));
%       end
%     end
%     
%     if idxB(i) ~= 1,
%       if compute_second_derivative,
%         [~,Jtmp,dJtmp] = obj.forwardKin(kinsol,idxB(i),xB(:,i));
%         J(I,:) = J(I,:) - Jtmp;
%         dJ(I,:) = dJ(I,:) - dJtmp;
%       else
%         [~,Jtmp] = obj.forwardKin(kinsol,idxB(i),xB(:,i));
%         J(I,:) = J(I,:) - Jtmp;
%       end
%     end
%   end
  
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
end
