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

      stateframe = CoordinateFrame([obj.model.name,'State'],2*obj.model.featherstone.NB,'x');
      joints={obj.model.body(~cellfun(@isempty,{obj.model.body.parent})).jointname}';
      stateframe = setCoordinateNames(stateframe,vertcat(joints,cellfun(@(a) [a,'dot'],joints,'UniformOutput',false)));
      obj = setStateFrame(obj,stateframe);
      obj = setOutputFrame(obj,stateframe);  % output = state
      
      if (length(obj.model.loop)>0)
        error('haven''t reimplemented position and velocity constraints yet'); 
      end
%      obj = obj.setNumPositionConstraints(2*length(obj.model.loop)+size([obj.model.body.ground_contact],2));
%      obj = obj.setNumVelocityConstraints(0);%size([obj.model.body.ground_contact],2));
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
    
    function v=constructVisualizer(obj)
      v = RigidBodyWRLVisualizer(obj.getStateFrame,obj.model);
    end
  end
  
end

