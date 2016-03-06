function [phi,B,JB,mu] = pairwiseContactConstraintsBV(obj,kinsol,body_indA,body_indB,body_collision_indA)

% Uses bullet to perform collision detection between rigid body A and rigid
% body B, and returns a single penetration constraint
%
% returns signed contact distance and a slightly different representation of
% the friction cone approx--basis vectors along the edges of the polyhedron, 
% positive combinations of which lie within the friction cone.
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
% @retval B friction polyhedron basis vectors
% @retval JB parameterization of the polyhedral approximation of the
%    friction cone, in joint coordinates
%    JB{k}(i,:) is the ith direction vector for the kth contact (of nC)
% @retval mu mu(i,1) is the coefficient of friction for the ith contact
% @ingroup Collision


if nargin > 4
  varargin = {kinsol,body_indA,body_indB,body_collision_indA};
else
  varargin = {kinsol,body_indA,body_indB};
end

if (nargout>2)
  [ptsA,ptsB,normal,J] = pairwiseContactTest(obj,varargin{:});
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
    d=obj.surfaceTangents(normal);
    m=length(d);
    B = cell(n_contact_pts,1);
    JB = cell(n_contact_pts,1);
    Bi = zeros(3,2*m);
    Ji = zeros(2*m,obj.num_positions);
  
    norm = sqrt(1+mu^2);% because normal and d are orthogonal, the norm has a simple form
    for i=1:n_contact_pts
      for k=1:m
        % todo: vectorize
        Bi(:,k) = (normal(:,i)+mu*d{k}(:,i)) / norm; 
        Bi(:,m+k) = (normal(:,i)-mu*d{k}(:,i)) / norm;
        if (nargout>1)
          Ji(k,:) = Bi(:,k)' * J((i-1)*3+(1:3),:);
          Ji(m+k,:) = Bi(:,m+k)' * J((i-1)*3+(1:3),:);
        end
      end
      B{i} = Bi;
      if (nargout>1)
        JB{i} = Ji';
      end
    end
  end
end
end