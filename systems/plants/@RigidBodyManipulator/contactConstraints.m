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

kinsol = doKinematics(obj,q,nargout>4);

contact_pos = zeros(3,obj.num_contacts)*q(1);  % q(1) to help TaylorVar
if (nargout>1) J = zeros(3*obj.num_contacts,obj.num_q)*q(1); end
count=0;
%      figure(1); clf;  % for debugging
for i=1:length(obj.body)
  nC = size(obj.body(i).contact_pts,2);
  if nC>0
    if (nargout>4)
      [contact_pos(:,count+(1:nC)),J(3*count+(1:3*nC),:),dJ(3*count+(1:3*nC),:)] = forwardKin(obj,kinsol,i,obj.body(i).contact_pts);
    elseif (nargout>1)
      [contact_pos(:,count+(1:nC)),J(3*count+(1:3*nC),:)] = forwardKin(obj,kinsol,i,obj.body(i).contact_pts);
    else
      contact_pos(:,count+(1:nC)) = forwardKin(obj,kinsol,i,obj.body(i).contact_pts);
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
  % recall that dphidx = normal'; n = dphidq = dphidx * dxdq
  % for a single contact, we'd have
  % n = normal'*J;
  % For vectorization, I just construct
  %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0],
  % etc, where each 0 is a 1x3 block zero, then multiply by J
  
  d = obj.surfaceTangents(normal);
  m=length(d);
  
  n = sparse(repmat(1:obj.num_contacts,3,1),1:3*obj.num_contacts,normal(:))*J;
  for k=1:m
    D{k} = sparse(repmat(1:obj.num_contacts,3,1),1:3*obj.num_contacts,d{k}(:))*J;
    if (nargout>4)
      dD{k} = reshape(sparse(repmat(1:obj.num_contacts,3,1),1:3*obj.num_contacts,t(:))*dJ,numel(n),[]);
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