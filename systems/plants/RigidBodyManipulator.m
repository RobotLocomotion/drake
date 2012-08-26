classdef RigidBodyManipulator < Manipulator
  % This class wraps the planar pieces of the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties (SetAccess=private,GetAccess=private)  
    model;     % RigidBodyModel object
  end
  
  methods
    function obj = RigidBodyManipulator(model)
      obj = obj@Manipulator(0,0);

      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        obj.model = RigidBodyModel(fullfile(pathname,filename));
      elseif ischar(model)
        obj.model = RigidBodyModel(model);
      elseif isa(model,'RigidBodyModel')
        obj.model = model;
      else
        error('model must be a RigidBodyModel or the name of a urdf file'); 
      end

      obj = obj.setNumInputs(size(obj.model.B,2));
      obj = obj.setNumDOF(obj.model.featherstone.NB);
      obj = obj.setNumOutputs(2*obj.model.featherstone.NB);
      
      if getNumInputs(obj)>0
        inputframe = CoordinateFrame([obj.model.name,'Input'],getNumInputs(obj));
        inputframe = setCoordinateNames(inputframe,{obj.model.actuator.name}');
        obj = setInputFrame(obj,inputframe);
      end

      if getNumStates(obj)>0
        stateframe = CoordinateFrame([obj.model.name,'State'],2*obj.model.featherstone.NB,'x');
        joints={obj.model.body(~cellfun(@isempty,{obj.model.body.parent})).jointname}';
        stateframe = setCoordinateNames(stateframe,vertcat(joints,cellfun(@(a) [a,'dot'],joints,'UniformOutput',false)));
        obj = setStateFrame(obj,stateframe);
        obj = setOutputFrame(obj,stateframe);  % output = state
      end
      
      if (length(obj.model.loop)>0)
        error('haven''t reimplemented position and velocity constraints yet'); 
      end
%      obj = obj.setNumPositionConstraints(2*length(obj.model.loop)+size([obj.model.body.ground_contact],2));
%      obj = obj.setNumVelocityConstraints(0);%size([obj.model.body.ground_contact],2));

      obj.joint_limit_min = [obj.model.body.joint_limit_min]';
      obj.joint_limit_max = [obj.model.body.joint_limit_max]';
      if (any(obj.joint_limit_min~=-inf) || any(obj.joint_limit_max~=inf))
        warning('Drake:RigidBodyManipulator:UnsupportedJointLimits','Joint limits are not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      obj.num_contacts = size([obj.model.body.contact_pts],2);
      if (obj.num_contacts>0)
        warning('Drake:RigidBodyManipulator:UnsupportedContactPoints','Contact is not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
    end
    
    function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd)
      m = obj.model.featherstone;
      
      if (nargout>3)
        % featherstone's HandCp with analytic gradients
        a_grav = [0;0;0;obj.model.gravity];
        
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
          dvdq{i} = zeros(6,m.NB)*q(1);
          dvdqd{i} = zeros(6,m.NB)*q(1);
          davpdq{i} = zeros(6,m.NB)*q(1);
          davpdqd{i} = zeros(6,m.NB)*q(1);
          dfvpdq{i} = zeros(6,m.NB)*q(1);
          dfvpdqd{i} = zeros(6,m.NB)*q(1);

          [ XJ, S{i} ] = jcalc( m.pitch(i), q(i) );
          dXJdq = djcalc(m.pitch(i), q(i));

          vJ = S{i}*qd(i);
          dvJdqd = S{i};
          
          Xup{i} = XJ * m.Xtree{i};
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
            
            avp{i} = Xup{i}*avp{j} + crm(v{i})*vJ;
            
            davpdq{i} = Xup{i}*davpdq{j};
            davpdq{i}(:,i) = davpdq{i}(:,i) + dXupdq{i}*avp{j};
            for k=1:m.NB,
              davpdq{i}(:,k) = davpdq{i}(:,k) + ...
                dcrm(v{i},vJ,dvdq{i}(:,k),zeros(6,1));
            end
            
            dvJdqd_mat = zeros(6,m.NB);
            dvJdqd_mat(:,i) = dvJdqd;
            davpdqd{i} = Xup{i}*davpdqd{j} + dcrm(v{i},vJ,dvdqd{i},dvJdqd_mat);
          end
          fvp{i} = m.I{i}*avp{i} + crf(v{i})*m.I{i}*v{i};
          dfvpdq{i} = m.I{i}*davpdq{i} + dcrf(v{i},m.I{i}*v{i},dvdq{i},m.I{i}*dvdq{i});
          dfvpdqd{i} = m.I{i}*davpdqd{i} + dcrf(v{i},m.I{i}*v{i},dvdqd{i},m.I{i}*dvdqd{i});
        end
        
        dC = zeros(m.NB,2*m.NB)*q(1);
        IC = m.I;				% composite inertia calculation
        dIC = cell(m.NB, m.NB);
        dIC = cellfun(@(a) zeros(6), dIC,'UniformOutput',false);
        
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
        dH = zeros(m.NB^2,2*m.NB)*q(1);
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
        
        B = obj.model.B;
        dB = zeros(m.NB*obj.num_u,2*m.NB);
      else
        [H,C] = HandC(m,q,qd,{},obj.model.gravity);
        C=C+m.damping'.*qd;
        B = obj.model.B;
      end
    end

    function [phi,n,D,mu] = contactConstraints(obj,q)
      % 
      % @retval phi  phi(i,1) is the signed distance from the contact
      % point on the robot to the closes object in the world.
      % @retval n the surface "normal vector", but in joint coordinates  (eq 3 in Anitescu97)
      %    n(i,:) is the normal for the ith contact
      % @retval D parameterization of the polyhedral approximation of the 
      %    friction cone, in joint coordinates (figure 1 from Stewart96)
      %    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
      % @retval mu mu(i,1) is the coefficient of friction for the ith contact 

      doKinematics(obj.model,q);
        
      contact_pos = zeros(3,obj.num_contacts);
      if (nargout>1) J = zeros(3*obj.num_contacts,obj.num_q); end
      count=0;
      for i=1:length(obj.model.body)
        body = obj.model.body(i);
        nC = size(body.contact_pts,2);
        if nC>0
          if (nargout>1)
            [contact_pos(:,count+(1:nC)),J(3*count+(1:3*nC),:)] = forwardKin(body,body.contact_pts);
          else
            contact_pos(:,count+(1:nC)) = forwardKin(body,body.contact_pts);
          end
          count = count + nC;
        end
      end
      
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
        end
        for k=(m/2+1):m
          D{k} = -D{k-m/2};
        end
        
        % the above is the vectorized version of this:
%        for i=1:obj.num_contacts
%          thisJ = J(3*(i-1)+(1:3),:);
%          n(i,:) = normal(:,i)'*thisJ;
%          for k=1:m
%            D{k}(i,:) = (cos(theta(k))*t1(:,i) + sin(theta(k))*t2(:,i))'*thisJ;
%          end
%        end
      end
    end

    function [pos,vel,normal,mu] = collisionDetect(obj,contact_pos)
      % for each column of contact_pos, find the closest point in the world
      % geometry, and it's (absolute) position and velocity, (unit) surface
      % normal, and coefficent of friction.
      
      % for now, just implement a ground height at y=0
      n = size(contact_pos,2);
      pos = [contact_pos(1:2,:);zeros(1,n)];
      vel = zeros(3,n); % static world assumption (for now)
      normal = [zeros(2,n); ones(1,n)];
      mu = ones(1,n);
    end
       
    function v=constructVisualizer(obj)
      v = RigidBodyWRLVisualizer(obj.getStateFrame,obj.model);
    end
  end
  
end

