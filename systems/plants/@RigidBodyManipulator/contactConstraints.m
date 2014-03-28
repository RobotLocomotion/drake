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

if compute_first_derivative
  nq = obj.getNumPositions;  
  d = obj.surfaceTangents(normal);
  nk = length(d);
  
  J = zeros(3*nC,nq);
  if compute_second_derivative,
    dJ = zeros(3*nC*nq,nq);
  end
  
  
  for i=1:nC,
    I = (1:3) + (i-1)*3;
    IdJ = (1:3*nq) + (i-1)*3*nq;
    
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
  
  if nargout > 9
    dn = reshape(sparse(indmat,1:3*n_contact_pts,normal(:))*dJ,numel(n),[]);
  end
end
end
%   
%   % the above is the vectorized version of this:
%   %        for i=1:obj.num_contacts
%   %          thisJ = J(3*(i-1)+(1:3),:);
%   %          n(i,:) = normal(:,i)'*thisJ;
%   %          for k=1:m
%   %            D{k}(i,:) = (cos(theta(k))*t1(:,i) + sin(theta(k))*t2(:,i))'*thisJ;
%   %          end
%   %        end
%   dn = reshape(sparse(indmat,1:3*n_contact_pts,normal(:))*dJ,numel(n),[]);
%   
%   % recall that dphidx = normal'; n = dphidq = dphidx * dxdq
%   % for a single contact, we'd have
%   % n = normal'*J;
%   % For vectorization, I just construct
%   %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0],
%   % etc, where each 0 is a 1x3 block zero, then multiply by J
%   
%   d = obj.surfaceTangents(normal);
%   m=length(d);
%   
%   indmat = repmat(1:n_contact_pts,3,1);
%   n = sparse(indmat,1:3*n_contact_pts,normal(:))*J;
%   D = cell(1,m);
%   dD = cell(1,m);
%   for k=1:m
%     D{k} = sparse(indmat,1:3*n_contact_pts,d{k}(:))*J;
%     if (nargout>4)
%       % note: this temporarily assumes that the normal does not change with contact_pos
%       dD{k} = reshape(sparse(indmat,1:3*n_contact_pts,d{k}(:))*dJ,numel(n),[]);
%     end
%   end
%   for k=(m+1):2*m
%     D{k} = -D{k-m};
%     if (nargout>4)
%       dD{k} = -dD{k-m};
%     end
%   end
%   
%   % the above is the vectorized version of this:
%   %        for i=1:obj.num_contacts
%   %          thisJ = J(3*(i-1)+(1:3),:);
%   %          n(i,:) = normal(:,i)'*thisJ;
%   %          for k=1:m
%   %            D{k}(i,:) = (cos(theta(k))*t1(:,i) + sin(theta(k))*t2(:,i))'*thisJ;
%   %          end
%   %        end
%   dn = reshape(sparse(indmat,1:3*n_contact_pts,normal(:))*dJ,numel(n),[]);
% end
% end
% 


% 
% function [phi,n,D,mu,dn,dD] = contactConstraints(obj,kinsol,body_idx,body_contacts)
% % @param body_idx is an array of body indexes
% % @param body_contacts is a cell array of vectors containing contact point indices
% % 
% % @retval phi  phi(i,1) is the signed distance from the contact
% % point on the robot to the closes object in the world.
% % @retval n the surface "normal vector", but in joint coordinates  (eq 3 in Anitescu97)
% %    n(i,:) is the normal for the ith contact
% % @retval D parameterization of the polyhedral approximation of the
% %    friction cone, in joint coordinates (figure 1 from Stewart96)
% %    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
% % @retval mu mu(i,1) is the coefficient of friction for the ith contact
% % @ingroup Collision
% 
% if nargin<3, body_idx = 1:length(obj.body); end
% if nargin<4
%   n_contact_pts = size([obj.body(body_idx).contact_pts],2);
%   varargin = {kinsol,body_idx};
% else
%   if isa(body_contacts,'cell')
%     n_contact_pts = sum(cellfun('length',body_contacts));
%   else
%     n_contact_pts = length(body_contacts);
%     body_contacts = {body_contacts};
%   end
%   varargin = {kinsol,body_idx,body_contacts};
% end
% 
% if (nargout>4)
%   [contact_pos,J,dJ] = contactPositions(obj,varargin{:});
% elseif (nargout>1)
%   [contact_pos,J] = contactPositions(obj,varargin{:});
% else
%   contact_pos = contactPositions(obj,varargin{:});
% end
% 
% %      axis equal;
% %      view(0,10);
% %      drawnow;  % for debugging
% 
% [pos,vel,normal,mu] = collisionDetect(obj,contact_pos);

% relpos = contact_pos - pos;
% phi = sum(relpos.*normal,1)';
% if (nargout>1)
% 
% end
% end