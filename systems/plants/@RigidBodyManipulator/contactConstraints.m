function [phi,n,D,mu,dn,dD] = contactConstraints(obj,kinsol,body_idx,body_contacts)
% @param body_idx is an array of body indexes
% @param body_contacts is a cell array of vectors containing contact point indices
% 
% @retval phi  phi(i,1) is the signed distance from the contact
% point on the robot to the closes object in the world.
% @retval n the surface "normal vector", but in joint coordinates  (eq 3 in Anitescu97)
%    n(i,:) is the normal for the ith contact
% @retval D parameterization of the polyhedral approximation of the
%    friction cone, in joint coordinates (figure 1 from Stewart96)
%    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
% @retval mu mu(i,1) is the coefficient of friction for the ith contact
% @ingroup Collision

if nargin<3, body_idx = 1:length(obj.body); end
if nargin<4
  n_contact_pts = size([obj.body(body_idx).contact_pts],2);
  varargin = {kinsol,body_idx};
else
  if isa(body_contacts,'cell')
    n_contact_pts = sum(cellfun('length',body_contacts));
  else
    n_contact_pts = length(body_contacts);
    body_contacts = {body_contacts};
  end
  varargin = {kinsol,body_idx,body_contacts};
end

if (nargout>4)
  [contact_pos,J,dJ] = contactPositions(obj,varargin{:});
elseif (nargout>1)
  [contact_pos,J] = contactPositions(obj,varargin{:});
else
  contact_pos = contactPositions(obj,varargin{:});
end

%      axis equal;
%      view(0,10);
%      drawnow;  % for debugging

[pos,vel,normal,mu] = collisionDetect(obj,contact_pos);

relpos = contact_pos - pos;
phi = sum(relpos.*normal,1)';
if (nargout>1)
  % recall that dphidx = normal'; n = dphidq = dphidx * dxdq
  % for a single contact, we'd have
  % n = normal'*J;
  % For vectorization, I just construct
  %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0],
  % etc, where each 0 is a 1x3 block zero, then multiply by J
  
  d = obj.surfaceTangents(normal);
  m=length(d);
  
  indmat = repmat(1:n_contact_pts,3,1);
  n = sparse(indmat,1:3*n_contact_pts,normal(:))*J;
  D = cell(1,m);
  dD = cell(1,m);
  for k=1:m
    D{k} = sparse(indmat,1:3*n_contact_pts,d{k}(:))*J;
    if (nargout>4)
      % note: this temporarily assumes that the normal does not change with contact_pos
      dD{k} = reshape(sparse(indmat,1:3*n_contact_pts,d{k}(:))*dJ,numel(n),[]);
    end
  end
  for k=(m+1):2*m
    D{k} = -D{k-m};
    if (nargout>4)
      dD{k} = -dD{k-m};
    end
  end
  
  % the above is the vectorized version of this:
  %        for i=1:obj.num_contacts
  %          thisJ = J(3*(i-1)+(1:3),:);
  %          n(i,:) = normal(:,i)'*thisJ;
  %          for k=1:m
  %            D{k}(i,:) = (cos(theta(k))*t1(:,i) + sin(theta(k))*t2(:,i))'*thisJ;
  %          end
  %        end
  if (nargout>4)
    dn = reshape(sparse(indmat,1:3*n_contact_pts,normal(:))*dJ,numel(n),[]);
  end
  
end
end