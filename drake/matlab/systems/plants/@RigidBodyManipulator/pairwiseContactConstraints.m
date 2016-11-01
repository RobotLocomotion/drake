function [phi,n,D,mu,dn,dD] = pairwiseContactConstraints(obj,kinsol,body_indA,body_indB,body_collision_indA)

% Uses bullet to perform collision detection between rigid body A and rigid
% body B, and returns a single penetration constraint
%
% @param kinsol  the output from doKinematics()
% @param body_indA  numerical index of rigid body A or 
%     (less efficient:) a rigidbody object
% @param body_indB  numerical index of rigid body B (or the rigidbody object)
%     if body_indB is -1, compute collisions with entire world
% @param body_collision_indA  index vector of collision objects on body A
%
% @retval phi  phi(i,1) is the signed distance from the contact
%      point on body A to the contact on body B the robot to the closes object in the world.
%      phi is set to 1 if not in contact
% @retval n the surface "normal vector", but in joint coordinates (eq 3 in Anitescu97)
%    n(i,:) is the normal for the ith contact
% @retval D parameterization of the polyhedral approximation of the
%    friction cone, in joint coordinates (figure 1 from Stewart96)
%    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
% @retval mu mu(i,1) is the coefficient of friction for the ith contact
% @ingroup Collision


if nargin > 4
  varargin = {kinsol,body_indA,body_indB,body_collision_indA};
else
  varargin = {kinsol,body_indA,body_indB};
end

if (nargout>4)
  [ptsA,ptsB,normal,JA,~,dJA] = pairwiseContactTest(obj,varargin{:});
elseif (nargout>1)
  [ptsA,ptsB,normal,JA] = pairwiseContactTest(obj,varargin{:});
else
  [ptsA,ptsB,normal] = pairwiseContactTest(obj,varargin{:});
end


%TMP hard code mu for now
mu=1.0;

if isempty(ptsA)
  phi = 1;
  n = zeros(1,getNumPositions(obj));
else
  relpos = ptsA-ptsB;
  s = sign(sum(relpos.*normal,1));
  phi = (sqrt(sum(relpos.^2,1)).*s)';
  n_contact_pts = length(phi);
  
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
    n = sparse(indmat,1:3*n_contact_pts,normal(:))*JA;
    D = cell(1,m);
    dD = cell(1,m);
    for k=1:m
      D{k} = sparse(indmat,1:3*n_contact_pts,d{k}(:))*JA;
      if (nargout>4)
        % note: this temporarily assumes that the normal does not change with contact_pos
        dD{k} = reshape(sparse(indmat,1:3*n_contact_pts,d{k}(:))*dJA,numel(n),[]);
      end
    end
    for k=(m+1):2*m
      D{k} = -D{k-m};
      if (nargout>4)
        dD{k} = -dD{k-m};
      end
    end

    if (nargout>4)
      dn = reshape(sparse(indmat,1:3*n_contact_pts,normal(:))*dJA,numel(n),[]);
    end  
  end
end  
end