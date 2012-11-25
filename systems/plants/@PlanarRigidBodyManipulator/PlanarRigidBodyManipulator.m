classdef PlanarRigidBodyManipulator < Manipulator
  % This class wraps the planar pieces of the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties (SetAccess=private,GetAccess=public)  
    model;     % PlanarRigidBodyModel object
    mex_model_ptr = 0;
  end
  
  methods
    function obj = PlanarRigidBodyManipulator(model)
      obj = obj@Manipulator(0,0);

      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        obj.model = PlanarRigidBodyModel(fullfile(pathname,filename));
      elseif ischar(model)
        obj.model = PlanarRigidBodyModel(model);
      elseif isa(model,'PlanarRigidBodyModel')
        obj.model = model;
      else
        error('model must be a PlanarRigidBodyModel or the name of a urdf file'); 
      end

      obj = obj.setNumInputs(size(obj.model.B,2));
      obj = obj.setNumDOF(obj.model.featherstone.NB);
      obj = obj.setNumOutputs(2*obj.model.featherstone.NB);
      
      if getNumInputs(obj)>0
        obj = setInputFrame(obj,constructInputFrame(obj.model));
      end
      
      if getNumStates(obj)>0
        stateframe = constructStateFrame(obj.model);
        obj = setStateFrame(obj,stateframe);
        obj = setOutputFrame(obj,stateframe);  % output = state
      end
        
      obj = obj.setNumPositionConstraints(2*length(obj.model.loop));
      
      obj.joint_limit_min = [obj.model.body.joint_limit_min]';
      obj.joint_limit_max = [obj.model.body.joint_limit_max]';
      if (any(obj.joint_limit_min~=-inf) || any(obj.joint_limit_max~=inf))
        warning('Drake:PlanarRigidBodyManipulator:UnsupportedJointLimits','Joint limits are not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      obj.num_contacts = size([obj.model.body.contact_pts],2);
      if (obj.num_contacts>0)
        warning('Drake:PlanarRigidBodyManipulator:UnsupportedContactPoints','Contact is not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      
      % warning:  this only works when there is a single planar rigid body model in use at any given time.
      % this simple logic attempts to guard against it.
      if checkDependency('eigen3_enabled')
        obj.mex_model_ptr = HandCpmex(struct(obj.model),obj.model.gravity);
      end
    end
    
    function deleteMex(obj)
      HandCpmex(obj.mex_model_ptr);
      obj.mex_model_ptr = 0;
    end
    
    function [Hqddot, dHqddot, phi, psi, dPhi, dPsi, H, dH] = implicitDynamics(obj,t,x,u,lambda,cLambda)
      % Compute the implicit dynamics for trajectory optimization.  This
      % function is fully implemented for joint limits and contact contraints
      % @param obj The Manipulator object
      % @param t Time
      % @param x State
      % @param u Input
      % @param lambda Non-contact constraint forces
      % @param cLambda Contact constraint forces
      % @return Hqddot H(q)*qddot from the dynamics equations
      % @return dHqddot d/dq and d/qdot of H*qddot
      % @return phi phi(q) the contact point positions
      % @return psi psi(q) the contact point velocities
      % @return dPhi dphi/dq
      % @return dPsi [dpsi/dq dpsi/dqdot]
      
      phi = [];
      psi = [];
      J = [];
      dPsi = [];
      % Compute H*qddot, one form of the implicit dynamics
      
      q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
      nContactConst = obj.num_contacts;
%       nonContactPConst = obj.num_bilateral_constraints - nContactConst;
      nonContactPConst = obj.getNumJointLimits;
      nUniPConstraints = nonContactPConst;
      dconstraint = 0;
      constraint = 0;
      nClutch = 0;
      
      [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd);
%       doKinematicsAndVelocities(obj.model,q,qd);
      if (obj.mex_model_ptr && isnumeric(q) && isnumeric(qd))
        doKinematicsmex(obj.mex_model_ptr,q,1)
      else
        doKinematics(obj.model,q,true);
      end
      
      if (nUniPConstraints > 0)
        [phi,J,dJ] = obj.jointLimits(q);
%         dJdqlambda = zeros(obj.num_q^2,1);
%         for i=1:obj.num_q^2,
%           dJdqlambda(i,1) = dJ(:,i)'*lambda(1:nUniPConstraints);
%         end
        dJdqlambda = dJ'*lambda(1:nUniPConstraints);
        dconstraint = [zeros(obj.num_q,1) reshape(dJdqlambda,obj.num_q,obj.num_q)...
          zeros(obj.num_q) zeros(obj.num_q,obj.num_u) J' ...
          zeros(obj.num_q,2*nContactConst + nonContactPConst - nUniPConstraints+nClutch)] + dconstraint;
        constraint = constraint + J'*lambda(1:nUniPConstraints);
      end
      
%       if (nonContactPConst - nUniPConstraints>0)
%         error('Not yet implemented in trunk version');
%         [phi,J,dJ] = obj.positionConstraints(q);
%         dJdqlambda = zeros(obj.num_q^2,1);
%         for i=1:obj.num_q^2,
%           dJdqlambda(i,1) = dJ(:,i)'*lambda(nUniPConstraints+1:end);
%         end
%         dconstraint = [zeros(obj.num_q,1) reshape(dJdqlambda,obj.num_q,obj.num_q)...
%           zeros(obj.num_q) zeros(obj.num_q,obj.num_u + nUniPConstraints) J' zeros(obj.num_q,2*nContactConst+nClutch)] + dconstraint;
%         constraint = constraint + J'*lambda(nUniPConstraints+1:end);
%       end
      
      if (nContactConst > 0)  
        [phi_n,n,D,mu,dn,dD,psi,dPsi,phi_f,dPhi] = contactConstraints(obj,q,qd);
%         [phi, psi, dPhi, dPsi, J, dJ]  = contactPositionsAndVelocities(obj,q,qd);
        
        %TODO: clean this up instead of just cramming into previous format
        psi = psi(1,:)';
        phi = zeros(2*length(phi_n),1);
        phi(1:2:end) = phi_f;
        phi(2:2:end) = phi_n;
        J = zeros(length(phi), obj.num_q);
        J(1:2:end,:) = D{1};
        J(2:2:end,:) = n;
        dJ = zeros(length(phi), obj.num_q^2);
        dJ(2:2:end,:) = reshape(dn,length(phi_n),[]);
        dJ(1:2:end,:) = reshape(dD{1},length(phi_n),[]);
        dPsi = dPsi(1:2:end,:);
        
%         dJdqlambda = zeros(obj.num_q^2,1)*x(1);
%         for i=1:obj.num_q^2,
%           dJdqlambda(i,1) = dJ(:,i)'*cLambda;
%         end
        dJdqlambda = dJ'*cLambda;
        dconstraint = [zeros(obj.num_q,1) reshape(dJdqlambda,obj.num_q,obj.num_q)...
          zeros(obj.num_q, obj.num_q + obj.num_u + nonContactPConst) J' zeros(obj.num_q, nClutch)] + dconstraint;
        constraint = constraint + J'*cLambda;
      end
      
      dHqddot = [zeros(obj.num_q,1) -dC B ...
        zeros(obj.num_q,nonContactPConst+2*nContactConst+nClutch)] + dconstraint;
      
%       if (~isempty(obj.model.passive_K) && ~isempty(obj.model.passive_B))
%         % determine which passive joints are active
%         active_ind = (q < obj.model.passive_max+1e-6).*(q > obj.model.passive_min-1e-6);
%         dHqddot(:,2:(1+2*obj.num_q)) = dHqddot(:,2:(1+2*obj.num_q)) - ...
%           [obj.model.passive_K*diag(active_ind) obj.model.passive_B*diag(active_ind)];
%         
%         passive_force = (-obj.model.passive_K*(q - obj.model.passive_nom) -...
%           obj.model.passive_B*qd).*active_ind + obj.model.passive_offset;
%         
%         Hqddot = B*u - C + constraint + passive_force;
%       else      
        Hqddot = B*u - C + constraint;
%         end
    end
    
    function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd)
      jsign = [obj.model.body(cellfun(@(a)~isempty(a),{obj.model.body.parent})).jsign]';
      q = jsign.*q;
      qd = jsign.*qd;
        
      if (nargout>3)
        if (obj.mex_model_ptr && isnumeric(q) && isnumeric(qd))
          [H,C,dH,dC] = HandCpmex(obj.mex_model_ptr,q,qd);
        else
          m = obj.model.featherstone;
          % featherstone's HandCp with analytic gradients
          a_grav = [0;obj.model.gravity];
          
          S = cell(m.NB,1);
          Xup = cell(m.NB,1);
          
          v = cell(m.NB,1);
          avp = cell(m.NB,1);
          
          %Derivatives
          dXupdq = cell(m.NB,1);
          dvdq = cell(m.NB,1);  %dvdq{i,j} is d/dq(j) v{i}
          dvdqd = cell(m.NB,1);
          davpdq = cell(m.NB,1);
          davpdqd = cell(m.NB,1);
          fvp = cell(m.NB,1);
          dfvpdq = cell(m.NB,1);
          dfvpdqd = cell(m.NB,1);
          
          
          for i = 1:m.NB
            dvdq{i} = zeros(3,m.NB)*q(1);
            dvdqd{i} = zeros(3,m.NB)*q(1);
            davpdq{i} = zeros(3,m.NB)*q(1);
            davpdqd{i} = zeros(3,m.NB)*q(1);
            dfvpdq{i} = zeros(3,m.NB)*q(1);
            dfvpdqd{i} = zeros(3,m.NB)*q(1);
            
            [ XJ, S{i} ] = jcalcp( m.jcode(i), q(i) );
            vJ = S{i}*qd(i);
            dvJdqd = S{i};
            
            Xup{i} = XJ * m.Xtree{i};
            dXJdq = djcalcp(m.jcode(i), q(i));
            dXupdq{i} = dXJdq * m.Xtree{i};
            
            if m.parent(i) == 0
              v{i} = vJ;
              dvdqd{i}(:,i) = dvJdqd;
              
              avp{i} = Xup{i} * -a_grav;
              davpdq{i}(:,i) = dXupdq{i} * -a_grav;
            else
              j = m.parent(i);
              v{i} = Xup{i}*v{j} + vJ;
              
              dvdq{i} = Xup{i}*dvdq{j};
              dvdq{i}(:,i) = dvdq{i}(:,i) + dXupdq{i}*v{j};
              
              dvdqd{i} = Xup{i}*dvdqd{j};
              dvdqd{i}(:,i) = dvdqd{i}(:,i) + dvJdqd;
              
              avp{i} = Xup{i}*avp{j} + crmp(v{i})*vJ;
              
              davpdq{i} = Xup{i}*davpdq{j};
              davpdq{i}(:,i) = davpdq{i}(:,i) + dXupdq{i}*avp{j};
              for k=1:m.NB,
                davpdq{i}(:,k) = davpdq{i}(:,k) + ...
                  dcrmp(v{i},vJ,dvdq{i}(:,k),zeros(3,1));
              end
              
              dvJdqd_mat = zeros(3,m.NB);
              dvJdqd_mat(:,i) = dvJdqd;
              davpdqd{i} = Xup{i}*davpdqd{j} + dcrmp(v{i},vJ,dvdqd{i},dvJdqd_mat);
            end
            fvp{i} = m.I{i}*avp{i} + crfp(v{i})*m.I{i}*v{i};
            dfvpdq{i} = m.I{i}*davpdq{i} + dcrfp(v{i},m.I{i}*v{i},dvdq{i},m.I{i}*dvdq{i});
            dfvpdqd{i} = m.I{i}*davpdqd{i} + dcrfp(v{i},m.I{i}*v{i},dvdqd{i},m.I{i}*dvdqd{i});
          end
          
          dC = zeros(m.NB,2*m.NB)*q(1);
          IC = m.I;				% composite inertia calculation
          dIC = cell(m.NB, m.NB);
          dIC = cellfun(@(a) zeros(3), dIC,'UniformOutput',false);
          
          for i = m.NB:-1:1
            C(i,1) = S{i}' * fvp{i};
            dC(i,:) = S{i}'*[dfvpdq{i} dfvpdqd{i}];
            if m.parent(i) ~= 0
              fvp{m.parent(i)} = fvp{m.parent(i)} + Xup{i}'*fvp{i};
              dfvpdq{m.parent(i)} = dfvpdq{m.parent(i)} + Xup{i}'*dfvpdq{i};
              dfvpdq{m.parent(i)}(:,i) = dfvpdq{m.parent(i)}(:,i) + dXupdq{i}'*fvp{i};
              dfvpdqd{m.parent(i)} = dfvpdqd{m.parent(i)} + Xup{i}'*dfvpdqd{i};
              
              IC{m.parent(i)} = IC{m.parent(i)} + Xup{i}'*IC{i}*Xup{i};
              for k=1:m.NB,
                dIC{m.parent(i),k} = dIC{m.parent(i),k} + Xup{i}'*dIC{i,k}*Xup{i};
              end
              dIC{m.parent(i),i} = dIC{m.parent(i),i} + ...
                dXupdq{i}'*IC{i}*Xup{i} + Xup{i}'*IC{i}*dXupdq{i};
            end
          end
          C=C+m.damping'.*qd;
          dC(:,m.NB+1:end) = dC(:,m.NB+1:end) + diag(m.damping);
          
          % minor adjustment to make TaylorVar work better.
          %H = zeros(m.NB);
          H=zeros(m.NB)*q(1);
          
          %Derivatives wrt q(k)
          dH = zeros(m.NB^2,m.NB)*q(1);
          for k = 1:m.NB
            for i = 1:m.NB
              fh = IC{i} * S{i};
              dfh = dIC{i,k} * S{i};  %dfh/dqk
              H(i,i) = S{i}' * fh;
              dH(i + (i-1)*m.NB,k) = S{i}' * dfh;
              j = i;
              while m.parent(j) > 0
                if j==k,
                  dfh = Xup{j}' * dfh + dXupdq{k}' * fh;
                else
                  dfh = Xup{j}' * dfh;
                end
                fh = Xup{j}' * fh;
                
                j = m.parent(j);
                H(i,j) = S{j}' * fh;
                H(j,i) = H(i,j);
                dH(i + (j-1)*m.NB,k) = S{j}' * dfh;
                dH(j + (i-1)*m.NB,k) = dH(i + (j-1)*m.NB,k);
              end
            end
          end
        end
        B = obj.model.B;
        dB = zeros(obj.num_q*obj.num_u,2*obj.num_q);
        dH = dH*diag(jsign);
        dC = diag(jsign)*dC*diag([jsign;jsign]);
      else
        if (obj.mex_model_ptr && isnumeric(q) && isnumeric(qd))
          [H,C] = HandCpmex(obj.mex_model_ptr,q,qd);
        else
          [H,C] = HandCp(obj.model.featherstone,q,qd,{},obj.model.gravity);
        end
        C=C+obj.model.featherstone.damping'.*qd;
        B = obj.model.B;
      end
      C = jsign.*C;
    end
    
    function phi = positionConstraints(obj,q)
      % so far, only loop constraints are implemented
      phi=loopConstraints(obj,q);
    end
    
    function phi = loopConstraints(obj,q)
      % handle kinematic loops
      % note: each loop adds two constraints 
      phi=[];
      jsign = [obj.model.body(cellfun(@(a)~isempty(a),{obj.model.body.parent})).jsign]';
      q = jsign.*q;
      
      for i=1:length(obj.model.loop)
        % for each loop, add the constraints on T1(q) and T2(q), // todo: finish this
        % where
        % T1 is the transformation from the least common ancestor to the
        % constraint in link1 coordinates
        % T2 is the transformation from the least common ancester to the
        % constraint in link2 coordinates
        loop=obj.model.loop(i);
        
        T1 = loop.T1;
        b=loop.body1;
        while (b~=loop.least_common_ancestor)
          TJ = Tjcalcp(b.jcode,q(b.dofnum));
          T1 = b.Ttree*TJ*T1;
          b = b.parent;
        end
        T2 = loop.T2;
        b=loop.body2;
        while (b~=loop.least_common_ancestor)
          TJ = Tjcalcp(b.jcode,q(b.dofnum));
          T2 = b.Ttree*TJ*T2;
          b = b.parent;
        end
        
        if (loop.jcode==1)  % pin joint adds constraint that the transformations must match in position at the origin
          phi = [phi; [1,0,0; 0,1,0]*(T1*[0;0;1] - T2*[0;0;1])];
        else
          error('not implemented yet');
        end
      end
    end
    
    function [x,J,dJ] = forwardKin(obj,body_ind,pts,use_mex_if_possible)
      if (nargin<4) use_mex_if_possible = true; end
      if nargout > 2
        if (use_mex_if_possible && obj.mex_model_ptr && isnumeric(pts))
          [x,J,dJ] = forwardKinmex(obj.mex_model_ptr,body_ind-1,pts);
        else
          [x,J,dJ] = forwardKin(obj.model.body(body_ind),pts);
        end
      elseif nargout > 1
        if (use_mex_if_possible && obj.mex_model_ptr && isnumeric(pts))
          [x,J] = forwardKinmex(obj.mex_model_ptr,body_ind-1,pts);
        else
          [x,J] = forwardKin(obj.model.body(body_ind),pts);
        end
        
      else
        if (use_mex_if_possible && obj.mex_model_ptr && isnumeric(pts))
          [x,J] = forwardKinmex(obj.mex_model_ptr,body_ind-1,pts);
        else
          [x,J] = forwardKin(obj.model.body(body_ind),pts);
        end
      end
    end
    
    function [v,dv] = forwardKinVel(obj,body_ind,pts,qd,use_mex_if_possible)
      if (nargin<4) use_mex_if_possible = true; end
      if nargout > 1
        if (use_mex_if_possible && obj.mex_model_ptr && isnumeric(qd))
          [v,dv] = forwardKinVelmex(obj.mex_model_ptr,body_ind-1,pts,qd);
        else
          [v,dv] = forwardKinVel(obj.model.body(body_ind),pts,qd);
        end
      else
        if (use_mex_if_possible && obj.mex_model_ptr && isnumeric(qd))
          v = forwardKinVelmex(obj.mex_model_ptr,body_ind-1,pts,qd);
        else
          v = forwardKinVel(obj.model.body(body_ind),pts,qd);
        end
      end
    end
    
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
      if (obj.mex_model_ptr && isnumeric(q) && (nargin<3 || isnumeric(qd)))
        doKinematicsmex(obj.mex_model_ptr,q,1);
        use_mex = true;
      else
        doKinematics(obj.model,q,nargout>4);
        use_mex = false;
      end

      contact_pos = zeros(2,obj.num_contacts)*q(1);  % *q(1) to help TaylorVar
      if (nargout>1) 
        J = zeros(2*obj.num_contacts,obj.num_q)*q(1); 
      end
      
      count=0;
%       nBodies = length(obj.model.body);
      nBodies = obj.model.featherstone.NB + 1;
      for i=1:nBodies;
%         body = obj.model.body(i);
        contact_pts = obj.model.body(i).contact_pts;
        nC = size(contact_pts,2);
        if nC>0
          if (nargout>4)
            [contact_pos(:,count+(1:nC)),J(2*count+(1:2*nC),:),dJ(2*count+(1:2*nC),:)] = forwardKin(obj,i,contact_pts,use_mex);
          elseif (nargout>1)
            [contact_pos(:,count+(1:nC)),J(2*count+(1:2*nC),:)] = forwardKin(obj,i,contact_pts,use_mex);
          else
            [contact_pos(:,count+(1:nC))] = forwardKin(obj,i,contact_pts,use_mex);
          end
          
          if (nargout>6)
            [contact_vel(:,count+(1:nC)),dv(2*count+(1:2*nC),:)] = forwardKinVel(obj,i,contact_pts,qd,use_mex);
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
      % if (abs(normal(2))>abs(normal(1))) t = [1,-n(1)/n(2)];  
      % else t = [-n(2)/n(1),1]; end
      % and the vectorized form is:
      t=normal; % initialize size
      ind=abs(normal(2,:))>abs(normal(1,:));
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
          dn = reshape(sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,normal(:))*dJ,prod(size(n)),[]);
          dD{1} = reshape(sparse(repmat(1:obj.num_contacts,2,1),1:2*obj.num_contacts,t(:))*dJ,prod(size(n)),[]);
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

    function [pos,vel,normal,mu] = collisionDetect(obj,contact_pos)
      % for each column of contact_pos, find the closest point in the world
      % geometry, and it's (absolute) position and velocity, (unit) surface
      % normal, and coefficent of friction.
      
      % for now, just implement a ground height at y=0
      n = size(contact_pos,2);
      pos = [contact_pos(1,:);zeros(1,n)];
      vel = zeros(2,n); % static world assumption (for now)
      normal = [zeros(1,n); ones(1,n)];
      mu = ones(1,n);
    end
    
    function v=constructVisualizer(obj)
      v = PlanarRigidBodyVisualizer(obj.getStateFrame,obj.model);
    end
    
    function v=constructWRLVisualizer(obj)
      options=struct();
      if (obj.num_contacts)
        options.ground=true;
      end
      v = RigidBodyWRLVisualizer(obj.getStateFrame,obj.model,options);
    end
  end
  
end

