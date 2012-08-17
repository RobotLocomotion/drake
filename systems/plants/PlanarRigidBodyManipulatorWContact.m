classdef PlanarRigidBodyManipulatorWContact < PlanarRigidBodyManipulator
  
  properties
    joint_limit_min=[];
    joint_limit_max=[];
    contact_pts=[];
  end
  
  methods
    function obj=PlanarRigidBodyManipulatorWContact(model)
      
      checkDependency('pathlcp_enabled');
      
      S = warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedJointLimits');
      obj = obj@PlanarRigidBodyManipulator(model);
      warning(S);

      obj.joint_limit_min = [obj.model.body.joint_limit_min]';
      obj.joint_limit_max = [obj.model.body.joint_limit_max]';
      
      if (obj.num_xcon>0)
        error('not implemented yet');
      end

      warning('Drake:PlanarRigidBodyManipulatorWContact:ShouldBeAHybridMode','This system is not really intended for use by itself, but as the mode in a HybridManipulator.  Consider instantiating a HybridManipulator instead');
    end
    
    function obj=setNumPositionConstraints(obj,n)
      if (n~=0)
        error('position constraints not implemented yet');
      end
    end
    function obj=setNumVelocityConstraints(obj,n)
      if (n~=0)
        error('velocity constraints not implemented yet');
      end
    end
    
    function constraint_force = computeConstraintForce(obj,q,qd,H,tau,Hinv)
      [phi,J,Jdot] = jointLimits(obj,q);
      phidot = J*qd;
      lambda=zeros(size(J,1),1);
      
      epsinv = 0; %1/.01;
      lcp_ind = (phi<=0 & phidot <= -epsinv*phi);
      
      if any(lcp_ind)
        Jlcp = J(lcp_ind,:);
        M = Jlcp*Hinv*Jlcp';
        b = Jdot(lcp_ind,:)*qd + Jlcp*Hinv*tau - 2*epsinv*phidot(lcp_ind) - epsinv^2*phi(lcp_ind);
        lambda(lcp_ind) = pathlcp(M,b);
      end
      constraint_force = J'*lambda;
    end
  end
  
  methods (Access=private)
    function [phi,J,Jdot] = jointLimits(obj,q)
      phi = [q-obj.joint_limit_min; obj.joint_limit_max-q]; phi=phi(~isinf(phi));
      J = [eye(obj.num_q); -eye(obj.num_q)];  
      J([obj.joint_limit_min==-inf;obj.joint_limit_max==inf],:)=[]; 
      Jdot = 0*J;
    end
  end
  
end