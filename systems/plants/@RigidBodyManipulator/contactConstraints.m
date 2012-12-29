function [phi,n,D,mu,dn,dD] = contactConstraints(obj,q,qd)
%
% @retval phi  phi(i,1) is the signed distance from the contact
% point on the robot to the closes object in the world.
% @retval n the surface "normal vector", but in joint coordinates  (eq 3 in Anitescu97)
%    n(i,:) is the normal for the ith contact
% @retval D parameterization of the polyhedral approximation of the
%    friction cone, in joint coordinates (figure 1 from Stewart96)
%    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
% @retval mu mu(i,1) is the coefficient of friction for the ith contact

use_mex = false;
if (obj.mex_model_ptr && isnumeric(q) && (nargin<3 || isnumeric(qd)))
  use_mex = true;
end
doKinematics(obj,q,nargout>4,use_mex); % checks for mex within this function now

contact_pos = zeros(3,obj.num_contacts)*q(1);  % q(1) to help TaylorVar
if (nargout>1) J = zeros(3*obj.num_contacts,obj.num_q)*q(1); end
count=0;
%      figure(1); clf;  % for debugging
for i=1:length(obj.body)
  nC = size(obj.body(i).contact_pts,2);
  if nC>0
    if (nargout>4)
      [contact_pos(:,count+(1:nC)),J(3*count+(1:3*nC),:),dJ(3*count+(1:3*nC),:)] = forwardKin(obj,i,obj.body(i).contact_pts,use_mex);
    elseif (nargout>1)
      [contact_pos(:,count+(1:nC)),J(3*count+(1:3*nC),:)] = forwardKin(obj,i,obj.body(i).contact_pts,use_mex);
    else
      contact_pos(:,count+(1:nC)) = forwardKin(obj,i,obj.body(i).contact_pts,use_mex);
    end
    count = count + nC;
    
    % for debugging
    %          ind = nchoosek(1:nC,2);
    %          for k=1:size(ind,1)
    %            line(contact_pos(1,ind(k,:)),contact_pos(2,ind(k,:)),contact_pos(3,ind(k,:)));
    %          end
    % end debugging
  end
end
%      axis equal;
%      view(0,10);
%      drawnow;  % for debugging

[pos,vel,normal,mu] = collisionDetect(obj,contact_pos);

relpos = contact_pos - pos;
s = sign(sum(relpos.*normal,1));
phi = (sqrt(sum(relpos.^2,1)).*s)';
if (nargout>1)
  
  %% compute tangent vectors, according to the description in the last paragraph of Stewart96, p.2678
  t1=normal; % initialize size
  % handle the normal = [0;0;1] case
  ind=(1-normal(3,:))<eps;  % since it's a unit normal, i can just check the z component
  t1(:,ind) = repmat([1;0;0],1,sum(ind));
  ind=~ind;
  % now the general case
  t1(:,ind) = cross(normal(:,ind),repmat([0;0;1],1,sum(ind)));
  t1 = t1./repmat(sqrt(sum(t1.^2,1)),3,1); % normalize
  
  t2 = cross(t1,normal);
  
  m = 4;  % must be an even number
  theta = (0:7)*2*pi/m;
  
  % recall that dphidx = normal'; n = dphidq = dphidx * dxdq
  % for a single contact, we'd have
  % n = normal'*J;
  % For vectorization, I just construct
  %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0],
  % etc, where each 0 is a 1x3 block zero, then multiply by J
  
  n = sparse(repmat(1:obj.num_contacts,3,1),1:3*obj.num_contacts,normal(:))*J;
  for k=1:m/2
    t=cos(theta(k))*t1 + sin(theta(k))*t2;
    D{k} = sparse(repmat(1:obj.num_contacts,3,1),1:3*obj.num_contacts,t(:))*J;
    if (nargout>4)
      dD{k} = reshape(sparse(repmat(1:obj.num_contacts,3,1),1:3*obj.num_contacts,t(:))*dJ,numel(n),[]);
    end
  end
  for k=(m/2+1):m
    D{k} = -D{k-m/2};
    if (nargout>4)
      dD{k} = -dD{k-m/2};
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
    dn = reshape(sparse(repmat(1:obj.num_contacts,3,1),1:3*obj.num_contacts,normal(:))*dJ,numel(n),[]);
  end
  
end
end