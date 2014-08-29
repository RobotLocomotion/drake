classdef ContactHybridRigidBodyManipulator < HybridDrakeSystem
  properties
    plant
  end
  methods
    function obj = ContactHybridRigidBodyManipulator(p,constraint_ind_sequence)
      obj = obj@HybridDrakeSystem(p.getNumInputs,p.getNumStates);
      obj = setInputFrame(obj,p.getInputFrame);
      obj = setOutputFrame(obj,p.getOutputFrame);
      obj.plant = p;
      
      M = length(constraint_ind_sequence);
      % double up to do n and j
      for i=1:M,
        obj = obj.addMode(ConstrainedRigidBodyManipulator(p,constraint_ind_sequence{i}));
      end
      
      for i=1:M-1,
        obj = obj.addTransition(i,@(t,x,u) obj.guard(t,x,u,constraint_ind_sequence{i},constraint_ind_sequence{i+1}),...
          transition,false,true,i+1);
        
      end
      
      function [g,dg] = guard_fun(obj,t,x,u,ind_in,ind_out)
        i_transition = ind_out(ind_out > ind_in);
        if any(i_transition)
          q = x(1:obj.plant.getNumPositions);
          [phi,normal,d,xA,xB,idxA,idxB,mu,n] = obj.plant.contactConstraints(q);
          assert(sum(i_transition)  == 1)
          g = phi(i_transition);
          dg = [0 n(i_transition,:) zeros(1,length(u))];
        else
          g = 0;
          dg = zeros(1,1+length(x)+length(u));
        end
      end
      
      function [xp,mode,status,dxp] = transition_fun(obj,mode,t,xm,u,ind_in,ind_out)
        i_transition = ind_out(ind_out > ind_in);
        if any(i_transition)
          q = x(1:obj.plant.getNumPositions);
          [phi,normal,d,xA,xB,idxA,idxB,mu,n] = obj.plant.contactConstraints(q);
          assert(sum(i_transition)  == 1)
          g = phi(i_transition);
          dg = [0 n(i_transition,:) zeros(1,length(u))];
        else
          xp = 0;
          dg = zeros(1,1+length(x)+length(u));
        end        
      end
    end
  end
end