function [phi,B,JB,mu] = contactConstraintsBV(obj,kinsol,body_idx,body_contacts)
% returns signed contact distance and a slightly different representation of
% the friction cone approx--basis vectors along the edges of the polyhedron, 
% positive combinations of which lie within the friction cone.
%
% @param body_idx is an array of body indexes
% @param body_contacts is a cell array of vectors containing contact point indices
% 
% @retval phi phi(i,1) is the signed distance from the contact
% point on the robot to the closes object in the world.
% @retval B friction polyhedron basis vectors
% @retval JB parameterization of the polyhedral approximation of the
%    friction cone, in joint coordinates
%    JB{k}(i,:) is the ith direction vector for the kth contact (of nC)
% @retval mu mu(i,1) is the coefficient of friction for the ith contact

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

if (nargout>2)
  [contact_pos,J] = contactPositions(obj,varargin{:});
else
  contact_pos = contactPositions(obj,varargin{:});
end

[pos,~,normal,mu] = collisionDetect(obj,contact_pos);

relpos = contact_pos - pos;
s = sign(sum(relpos.*normal,1));
phi = (sqrt(sum(relpos.^2,1)).*s)';
if (nargout>1)
  d=obj.surfaceTangents(normal);
  m=length(d);
  B = cell(n_contact_pts,1);
  JB = cell(n_contact_pts,1);
  Bi = zeros(3,2*m);
  Ji = zeros(2*m,obj.num_q);
  
  for i=1:n_contact_pts
    norm = sqrt(1+mu(i)^2);% because normal and d are orthogonal, the norm has a simple form
    for k=1:m
      % todo: vectorize
      Bi(:,k) = (normal(:,i)+mu(i)*d{k}(:,i)) / norm; 
      Bi(:,m+k) = (normal(:,i)-mu(i)*d{k}(:,i)) / norm;
      if (nargout>2)
        Ji(k,:) = Bi(:,k)' * J((i-1)*3+(1:3),:);
        Ji(m+k,:) = Bi(:,m+k)' * J((i-1)*3+(1:3),:);
      end
    end
    B{i} = Bi;
    if (nargout>2)
      JB{i} = Ji';
    end
  end
end
end

