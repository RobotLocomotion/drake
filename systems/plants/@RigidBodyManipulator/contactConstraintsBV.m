function [phi,B,JB,mu,normal] = contactConstraintsBV(obj,varargin)
% function [phi,B,JB,mu] = contactConstraintsBV(obj,kinsol,allow_multiple_contacts,active_collision_options)
% a variation on the contactConstraints function that uses a different
% representation of the friction cone--basis vectors are the extreme rays
% of the cone (no basis for the normal force)
%
% @retval phi (m x 1) Vector of gap function values (typically contact distance), for m possible contacts
% @retval B {m}(3 x 2k) friction polyhedron basis vectors
% @retval JB {m}{2k x n} parameterization of the polyhedral approximation of the
%    friction cone, in joint coordinates
%    JB{k}(i,:) is the ith direction vector for the kth contact (of nC)
% @retval mu (m x 1) Coefficients of friction
% @retval normal (3 x m) Contact normal vector in world coordinates, points from B to A
% @param See contactConstraints for argument list description

if nargout > 2,
  [phi,normal,d,~,~,~,~,mu,n,D] = contactConstraints(obj,varargin{:});
else
  [phi,normal,d] = contactConstraints(obj,varargin{:});
end

if nargout > 1,
  nk = length(d);
  nC = length(phi);
  nq = obj.getNumPositions;
  
  % normalize vectors
  normalize_mat = diag(1./sqrt(1 + mu.^2));
  
  B_mat = zeros(3,2*nk*nC);
  
  if nargout > 2
    JB_mat = zeros(2*nk,nq*nC);
  end
  
  % Awkwardly reshaping everything because the old contactConstraintsBV had
  % a different output format than contactConstraints, maintaining
  % backwards compatability.
  for k=1:nk,
    I = k - 1 + (1:2*nk:2*nk*nC);
    B_mat(:,I) = (normal + d{k}*diag(mu))*normalize_mat;
    B_mat(:,I+nk) = (normal - d{k}*diag(mu))*normalize_mat;
    
    if nargout > 2
      JB_mat(k,:) = reshape((n' + D{k}'*diag(mu))*normalize_mat,1,[]);
      JB_mat(k+nk,:) = reshape((n' - D{k}'*diag(mu))*normalize_mat,1,[]);
    end
  end
  B = mat2cell(B_mat,3,2*nk*ones(nC,1));  
  if nargout > 2
    JB = mat2cell(JB_mat,2*nk,nq*ones(nC,1));
  end
end
end
