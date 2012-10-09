classdef HybridRigidBodyMode < RigidBodyManipulator

  properties
    joint_limit_state
    contact_state
  end
  
  methods
    function obj = HybridRigidBodyMode(model,joint_limit_state,contact_state,in_frame,state_frame,out_frame)
      S = warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@RigidBodyManipulator(model);
      warning(S);
      
      obj.joint_limit_state = joint_limit_state;
      obj.contact_state = contact_state;

      if (obj.num_position_constraints || obj.num_velocity_constraints)
        error('still need to handle the case whether there are other constraints involved, too');
      end
      
      obj = setNumPositionConstraints(obj,sum(joint_limit_state~=0)+sum(contact_state>0));
      obj = setNumVelocityConstraints(obj,sum(contact_state==1));
      
      % a reminder that somwhere I need to implement the sliding friction
      if any(contact_state>1) error('not implemented yet'); end
      
      if (nargin>3) obj = setInputFrame(obj,in_frame); end
      if (nargin>4) obj = setStateFrame(obj,state_frame); end
      if (nargin>5) obj = setOutputFrame(obj,out_frame); end
    end
    
%    function [phi,J,dJ] = positionConstraints(obj,q)
    function phi = positionConstraints(obj,q)
      phiC = contactConstraints(obj,q);
      phi = [q(obj.joint_limit_state==1) - obj.joint_limit_min(obj.joint_limit_state==1); ...
             obj.joint_limit_max(obj.joint_limit_state==2) - q(obj.joint_limit_state==2); ...
             phiC(obj.contact_state>0)];
    end
    
%    function [psi,J] = velocityConstraints(obj,q,qd)
    function psi = velocityConstraints(obj,q,qd)
      [phi,n,D] = contactConstraints(obj,q);
      psi = vertcat(D{:}(obj.contact_state==1,1)*qd);
    end
    
    function [x,success] = resolveConstraints(obj,x0,v)
      if (nargin<3) v=[]; end
      [x,success] = resolveConstraints@SecondOrderSystem(obj,x0,v);
    end
    
  end
end
