classdef RigidBodyTorsionalSpring < RigidBodyForceElement
%  Adds torsionals linear springs, with 
%      torque = k*(rest_angle - current_angle),
%  to rigid body manipulators.  
  
  properties
    joint 
    rest_angle=0
    k=0
  end
  
  methods
    function f_ext = computeSpatialForce(obj,manip,q,qd)
      theta = q(manip.body(obj.joint).dofnum);
      torque = obj.k*(obj.rest_angle - theta);
      f_ext = sparse(6,getNumBodies(manip));

      % note, because featherstone coordinates do everything about the
      % z-axis, the below is actually equivalent to
      % f_ext(:,obj.joint) = manip.body(obj.joint).X_joint_to_body'*[zeros(3,1);torque*manip.body(obj.joint).joint_axis];
      f_ext(:,obj.joint) = [zeros(2,1);torque;zeros(3,1)]; 
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.joint = map_from_old_to_new(obj.joint);
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.joint == body_ind)
        error('removed joint with a torsional spring.  need to handle this case');
      end
    end
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,robotnum,node,options)
      obj = RigidBodyTorsionalSpring();
      
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');
      obj.name = name;
      
      if node.hasAttribute('rest_angle')
        obj.rest_angle = parseParamString(model,robotnum,char(node.getAttribute('rest_angle')));
      end
      if node.hasAttribute('stiffness')
        obj.k = parseParamString(model,robotnum,char(node.getAttribute('stiffness')));
      end
      
      jointNode = node.getElementsByTagName('joint').item(0);
      obj.joint = findJointInd(model,char(jointNode.getAttribute('name')),robotnum);
      
      assert(numel(model.body(obj.joint).dofnum)==1 && model.body(obj.joint).pitch==0,'Torsional springs are currnetly only supported for pin joints (aka, continuous and rotational joints)');
    end
  end
end

