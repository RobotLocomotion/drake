function [phi,n,D,mu,dn,dD,psi,dPsi,phi_f,dPhi] = contactConstraints(obj,q,qd)
%
% @retval phi  phi(i,1) is the signed distance from the contact
% point on the robot to the closes object in the world.
% @retval n the surface "normal vector", but in joint coordinates  (eq 3 in Anitescu97)
%    n(i,:) is the normal for the ith contact
% @retval D parameterization of the polyhedral approximation of the
%    friction cone, in joint coordinates (figure 1 from Stewart96)
%    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
% @retval mu mu(i,1) is the coefficient of friction for the ith contact
% @retval psi the planar velocity of the contacts (x1,z1,x2,z2,...)

if (nargout > 6)
  contact_vel = zeros(2,obj.num_contacts)*q(1);  % *q(1) to help TaylorVar
  dv = zeros(2*obj.num_contacts,2*obj.num_q)*q(1);
end

kinsol = doKinematics(obj,q,nargout>4);

contact_pos = zeros(2,obj.num_contacts)*q(1);  % *q(1) to help TaylorVar
if (nargout>1)
  J = zeros(2*obj.num_contacts,obj.num_q)*q(1);
end

count=0;
%       nBodies = length(obj.model.body);
nBodies = obj.featherstone.NB + 1;
for i=1:nBodies;
  %         body = obj.model.body(i);
  contact_pts = obj.body(i).contact_pts;
  nC = size(contact_pts,2);
  if nC>0
    if (nargout>4)
      [contact_pos(:,count+(1:nC)),J(2*count+(1:2*nC),:),dJ(2*count+(1:2*nC),:)] = forwardKin(obj,kinsol,i,contact_pts);
    elseif (nargout>1)
      [contact_pos(:,count+(1:nC)),J(2*count+(1:2*nC),:)] = forwardKin(obj,kinsol,i,contact_pts);
    else
      [contact_pos(:,count+(1:nC))] = forwardKin(obj,kinsol,i,contact_pts);
    end
    
    if (nargout>6)
      [contact_vel(:,count+(1:nC)),dv(2*count+(1:2*nC),:)] = forwardKinVel(obj,kinsol,i,contact_pts,qd);
    end
    
    count = count + nC;
  end
end

[pos,vel,normal,mu] = collisionDetect(obj,contact_pos);

% note: without asking the collision detector for curvature of the
% surface, the best we can do is assume that the world is locally flat.
% e.g. dnormal/dcontact_pos = 0;

relpos = contact_pos - pos;
s = sign(sum(relpos.*normal,1));
%       phi = (sqrt(sum(relpos.^2,1)).*s)'; %replaced this with normal
%       distance

t = obj.surfaceTangents(normal); t=t{1};

phi = sum(relpos.*normal)';
phi_f = sum(relpos.*t)';
if (nargout>1)
  % recall that dphidx = normal'; n = dphidq = dphidx * dxdq
  % for a single contact, we'd have
  % n = normal'*J;
  % For vectorization, I just construct
  %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0],
  % etc, where each 0 is a 1x3 block zero, then multiply by J
  
  n = sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,normal(:))*J;
  D{1} = sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,t(:))*J;
  D{2} = -D{1};
  
  % the above is the vectorized version of this:
  %        for i=1:obj.num_contacts
  %          thisJ = J(2*(i-1)+(1:2),:);
  %          n(i,:) = normal(:,i)'*thisJ;
  %          D{1}(i,:) = t(:,i)'*thisJ;
  %          D{2}(i,:) = -t(:,i)'*thisJ;
  %        end
  
  if (nargout>4)
    % dnormal/dx = 0 (see discussion above), so the gradients are simply:
    dn = reshape(sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,normal(:))*dJ,numel(n),[]);
    dD{1} = reshape(sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,t(:))*dJ,numel(n),[]);
    dD{2} = -dD{1};
  end
  %TODO: if the object being collided with is movable, then need it's
  %gradient as well here
  if (nargout>6)
    rel_vel = contact_vel - vel;
    % normal velocities sum(contact_vel.*t)
    psi = [sum(rel_vel.*t);sum(rel_vel.*normal)];
    dPsi = zeros(size(dv));
    dPsi(1:2:end,:) = reshape(sum(reshape(repmat(t(:),1,2*obj.num_q).*dv,2,[]),1),size(t,2),[]);
    dPsi(2:2:end,:) = reshape(sum(reshape(repmat(normal(:),1,2*obj.num_q).*dv,2,[]),1),size(t,2),[]);
    dPhi = zeros(2*length(phi), obj.num_q);
    dPhi(1:2:end,:) = D{1};
    dPhi(2:2:end,:) = n;
  end
end
end
