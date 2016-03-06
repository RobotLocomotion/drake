classdef Gripper2D_gcs < Gripper2D
  
  properties (SetAccess=protected,GetAccess=public)
    
  end
  
  methods
    function obj = Gripper2D_gcs()
      obj = obj@Gripper2D;
      obj.num_contacts = 2*(obj.num_contacts-1) + 1; %add ground contact
    end
    
    function [phi_n,n,D,mu,dn,dD,psi,dPsi,phi_f,dPhi] = contactConstraints(obj,q,qd)
      [phi, psi, dPhi, dPsi, J, dJ, psi_full] = contactPositionsAndVelocities(obj,q,qd);
      n = J(2:2:end,:);
      D{1} = J(1:2:end,:);
      D{2} = -D{1};
      mu = 1;
      phi_f = phi(1:2:end);
      phi_n = phi(2:2:end);
      
      dn = reshape(dJ(2:2:end,:),obj.num_positions*length(phi_n),obj.num_positions);
      dD{1} = reshape(dJ(1:2:end,:),obj.num_positions*length(phi_n),obj.num_positions);
      dD{2} = -dD{1};
      
      psi = reshape(psi_full,2,[]);
      
      %concatenate the original ground contacts
      [phi_n2,n2,D2,~,dn2,dD2,psi2,dPsi2,phi_f2] = groundContactConstraints(obj,q,qd,(obj.num_contacts-1)/2);
      
%       phi_n = 0*phi_n;
%       n = 0*n;
%       D{1} = 0*D{1};
%       D{2} = 0*D{2};      
%       dn = 0*dn;
%       psi = 0*psi;
%       dPsi = 0*dPsi;
%       phi_f = 0*phi_f;      
%       dD{1} = 0*dD{1};
%       dD{2} = 0*dD{2};
      
%       phi_n2 = 0*phi_n2;
%       n2 = 0*n2;
%       D2{1} = 0*D2{1};
%       D2{2} = 0*D2{2};      
%       dn2 = 0*dn2;
%       psi2 = 0*psi2;
%       dPsi2 = 0*dPsi2;
%       phi_f2 = 0*phi_f2;      
%       dD2{1} = 0*dD2{1};
%       dD2{2} = 0*dD2{2};
      
      n = [n;n2];
      D{1} = [D{1};D2{1}];
      D{2} = [D{2};D2{2}];
      dn = reshape([reshape(dn,length(phi_n),obj.num_positions^2); reshape(dn2,length(phi_n2),obj.num_positions^2)],[],obj.num_positions);
      dD{1} = reshape([reshape(dD{1},length(phi_n),obj.num_positions^2); reshape(dD2{1},length(phi_n2),obj.num_positions^2)],[],obj.num_positions);
      dD{2} = reshape([reshape(dD{2},length(phi_n),obj.num_positions^2); reshape(dD2{2},length(phi_n2),obj.num_positions^2)],[],obj.num_positions);
%       dD{1} = [dD{1};dD2{1}];
%       dD{2} = [dD{2};dD2{2}];
      dPsi = [dPsi;dPsi2];
      phi_f = [phi_f;phi_f2];
      psi = [psi psi2];
      phi_n = [phi_n;phi_n2];
      
      J2 = zeros(2*length(phi_n2), obj.num_positions);
      J2(1:2:end,:) = D2{1};
      J2(2:2:end,:) = n2;
      dPhi = [dPhi;J2];
    end
    
    function [phi,n,D,mu,dn,dD,psi,dPsi,phi_f] = groundContactConstraints(obj,q,qd,nObjContacts)
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
        contact_vel = zeros(2,nObjContacts)*q(1);  % *q(1) to help TaylorVar  
        dv = zeros(2*nObjContacts,2*obj.num_positions)*q(1);
      else
      end
      kinsol = doKinematicsmex(obj.mex_model_ptr,q,1); % FIXME

      contact_pos = zeros(2,nObjContacts)*q(1);  % *q(1) to help TaylorVar
      if (nargout>1) 
        J = zeros(2*nObjContacts,obj.num_positions)*q(1); 
      end
      
      count=0;
%       nBodies = length(obj.model.body);
      nBodies = length(obj.body);
      for i=1:nBodies;
%         body = obj.model.body(i);
        contact_pts = obj.model.body(i).contact_pts;
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
      %% compute a tangent vector, t
      % for each n, it looks like:
      % if (normal(2)>normal(1)) t = [1,-n(1)/n(2)];
      % else t = [-n(2)/n(1),1]; end
      % and the vectorized form is:
      t=normal; % initialize size
      ind=normal(2,:)>normal(1,:);
      t(:,ind) = [ones(1,sum(ind));-normal(1,ind)./normal(2,ind)];
      ind=~ind;
      t(:,ind) = [-normal(2,ind)./normal(1,ind); ones(1,sum(ind))];
      t = t./repmat(sqrt(sum(t.^2,1)),2,1); % normalize
      
      phi = sum(relpos.*normal)';
      phi_f = sum(relpos.*t)';
      if (nargout>1)
        % recall that dphidx = normal'; n = dphidq = dphidx * dxdq
        % for a single contact, we'd have
        % n = normal'*J;
        % For vectorization, I just construct
        %  [normal(:,1)' 0 0 0 0; 0 normal(:,2)' 0 0 0; 0 0 normal(:,3') 0 0], 
        % etc, where each 0 is a 1x3 block zero, then multiply by J

        n = sparse(repmat(1:nObjContacts,2,1),1:2*nObjContacts,normal(:))*J;
        D{1} = sparse(repmat(1:nObjContacts,2,1),1:2*nObjContacts,t(:))*J;
        D{2} = -D{1};
        
        % the above is the vectorized version of this:
%        for i=1:nObjContacts
%          thisJ = J(2*(i-1)+(1:2),:);
%          n(i,:) = normal(:,i)'*thisJ;
%          D{1}(i,:) = t(:,i)'*thisJ;
%          D{2}(i,:) = -t(:,i)'*thisJ;
%        end

        if (nargout>4)
          % dnormal/dx = 0 (see discussion above), so the gradients are simply:
          dn = reshape(sparse(repmat(1:nObjContacts,2,1),1:2*nObjContacts,normal(:))*dJ,prod(size(n)),[]);
          dD{1} = reshape(sparse(repmat(1:nObjContacts,2,1),1:2*nObjContacts,t(:))*dJ,prod(size(n)),[]);
          dD{2} = -dD{1};
        end
        %TODO: if the object being collided with is movable, then need it's
        %gradient as well here
        if (nargout>6)
          rel_vel = contact_vel - vel;
          % normal velocities sum(contact_vel.*t)
          psi = [sum(rel_vel.*t);sum(rel_vel.*normal)];
          dPsi = zeros(size(dv));
          dPsi(1:2:end,:) = reshape(sum(reshape(repmat(t(:),1,2*obj.num_positions).*dv,2,[]),1),size(t,2),[]);
          dPsi(2:2:end,:) = reshape(sum(reshape(repmat(normal(:),1,2*obj.num_positions).*dv,2,[]),1),size(t,2),[]);
        end
      end
    end
  end
  
end