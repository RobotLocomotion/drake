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
nC = length(phi);

% For now, all coefficients of friction are 1
mu = ones(nC,1);

d = obj.surfaceTangents(normal);
if compute_first_derivative
  nq = obj.getNumPositions;  
  nk = length(d);
  
  J = zeros(3*nC,nq);
  if compute_second_derivative,
    dJ = zeros(3*nC,nq*nq);
  end
  
  
  for i=1:nC,
    I = (1:3) + (i-1)*3;
%     IdJ = (1:3*nq) + (i-1)*3*nq;
    IdJ = I;
    
    % For each of the two bodies, if it is a real body (i.e. idx != 0),
    % then add compute the relevent jacobians in joint coordinates
    if idxA(i) ~= 0,
      if compute_second_derivative,
        [~,J(I,:),dJ(IdJ,:)] = obj.forwardKin(kinsol,idxA(i),xA(:,i));
      else
        [~,J(I,:)] = obj.forwardKin(kinsol,idxA(i),xA(:,i));
      end
    end
    
    if idxB(i) ~= 0,
      if compute_second_derivative,
        [~,Jtmp,dJtmp] = obj.forwardKin(kinsol,idxB(i),xB(:,i));
        J(I,:) = J(I,:) - Jtmp;
        dJ(IdJ,:) = dJ(IdJ,:) - dJtmp;
      else
        [~,Jtmp] = obj.forwardKin(kinsol,idxB(i),xB(:,i));
        J(I,:) = J(I,:) - Jtmp;
      end
    end
  end
  
  indmat = repmat(1:nC,3,1);
  n = sparse(indmat,1:3*nC,normal(:))*J;
  D = cell(1,nk);
  dD = cell(1,nk);
  for k=1:nk,
    D{k} = sparse(indmat,1:3*nC,d{k}(:))*J;
    if compute_second_derivative
      % note: this temporarily assumes that the normal does not change with contact_pos
      dD{k} = reshape(sparse(indmat,1:3*nC,d{k}(:))*dJ,numel(n),[]);
    end
  end
  for k=(nk+1):2*nk
    D{k} = -D{k-nk};
    if compute_second_derivative
      dD{k} = -dD{k-nk};
    end
  end
  
  if compute_second_derivative
    dn = reshape(sparse(indmat,1:3*nC,normal(:))*dJ,numel(n),[]);
  end
end
end