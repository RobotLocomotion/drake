classdef ConstrainedRigidBodyManipulator < SecondOrderSystem
  properties
    constraint_ind
    plant
  end
  
  methods
    function obj = ConstrainedRigidBodyManipulator(p,constraint_ind)
      obj = obj@SecondOrderSystem(p.getNumPositions, p.getNumInputs, p.isTI);
      obj.constraint_ind = constraint_ind;
      obj.plant = p;
      
      obj = obj.setStateFrame(p.getStateFrame);
      obj = obj.setOutputFrame(p.getOutputFrame);
      obj = obj.setInputFrame(p.getInputFrame);
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)      
      [H,C,B] = obj.plant.manipulatorDynamics(q,qd);
      Hinv = inv(H);
      if (size(obj.constraint_ind) > 0)

        [phi_n,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.plant.contactConstraints(q);
        
        phi = zeros(2*length(phi_n),1);
        %   phi(1:2:end) = phi_f;
        phi(2:2:end) = phi_n;
        
        J = zeros(length(phi), obj.num_q);
        J(1:2:end,:) = D{1};
        J(2:2:end,:) = n;
        
        dJ = zeros(length(phi), obj.num_q^2);
        dJ(2:2:end,:) = reshape(dn,length(phi_n),[]);
        dJ(1:2:end,:) = reshape(dD{1},length(phi_n),[]);
        
        %   phi_sub = phi_full(constraint_ind);
        Jdotqd = dJ(obj.constraint_ind,:)*reshape(qd*qd',obj.num_q^2,1);
        J_sub = J(obj.constraint_ind,:);
        
        if isempty(u)
          constraint_force = -J_sub'*(pinv(J_sub*Hinv*J_sub')*(J_sub*Hinv*(-C) + Jdotqd));
        else
          constraint_force = -J_sub'*(pinv(J_sub*Hinv*J_sub')*(J_sub*Hinv*(B*u-C) + Jdotqd));
        end
      else
        constraint_force = 0;
      end
      
      if isempty(u)
        qdd = Hinv*(constraint_force - C);
      else
        qdd = Hinv*(B*u + constraint_force - C);
      end
    end
    
    function v = constructVisualizer(obj)
      v = obj.plant.constructVisualizer;
    end
    
    function v = constraintVelocity(obj,q,qd)
      if (size(obj.constraint_ind) > 0)        
        [phi_n,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.plant.contactConstraints(q);
        
        phi = zeros(2*length(phi_n),1);
        %   phi(1:2:end) = phi_f;
        phi(2:2:end) = phi_n;
        
        J = zeros(length(phi), obj.num_q);
        J(1:2:end,:) = D{1};
        J(2:2:end,:) = n;
        
        dJ = zeros(length(phi), obj.num_q^2);
        dJ(2:2:end,:) = reshape(dn,length(phi_n),[]);
        dJ(1:2:end,:) = reshape(dD{1},length(phi_n),[]);
        
        %   phi_sub = phi_full(constraint_ind);
        Jdotqd = dJ(obj.constraint_ind,:)*reshape(qd*qd',obj.num_q^2,1);
        v = Jdotqd;
      else
        v = [];
      end
    end
  end
end