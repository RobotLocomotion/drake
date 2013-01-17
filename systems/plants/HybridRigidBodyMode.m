classdef HybridRigidBodyMode < PlanarRigidBodyManipulator

  properties (SetAccess=protected)
    joint_limit_state
    contact_state
  end
  
  methods
    function obj = HybridRigidBodyMode(urdf_filename,joint_limit_state,contact_state,options)
      if (nargin<4) options=struct(); end
      S = warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@PlanarRigidBodyManipulator(urdf_filename,options);
      warning(S);
      
      sizecheck(joint_limit_state,[obj.num_q,1]);
      obj.joint_limit_state = joint_limit_state;
      
      sizecheck(contact_state,[obj.num_contacts,1]);
      obj.contact_state = contact_state;

      % a reminder that somwhere I need to implement the sliding friction
      if any(contact_state>1) error('not implemented yet'); end
      
      obj = compile(obj);
    end
    
    function obj = compile(obj)
      obj = compile@PlanarRigidBodyManipulator(obj);
      
      if (obj.num_position_constraints || obj.num_velocity_constraints)
        error('still need to handle the case whether there are other constraints involved, too');
      end
      
      obj = setNumPositionConstraints(obj,sum(obj.joint_limit_state~=0)+sum(obj.contact_state>0));
      obj = setNumVelocityConstraints(obj,sum(obj.contact_state==1));
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
      [phi,n,D] = contactConstraints(obj,q,qd);
      psi = D{1}(obj.contact_state==1,:)*qd;
    end
    
    function [x,success] = resolveConstraints(obj,x0,v)
      if (nargin<3) v=[]; end
      [x,success] = resolveConstraints@SecondOrderSystem(obj,x0,v);
    end
    
  end
end
