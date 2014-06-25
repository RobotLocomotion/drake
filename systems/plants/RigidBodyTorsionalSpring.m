classdef RigidBodyTorsionalSpring < RigidBodyForceElement
  
  properties
    joint 
    rest_angle=0
    k=0
  end
  
  methods
    function f_ext = computeSpatialForce(obj,manip,q,qd)
      theta = q(manip.body(obj.joint).dofnum);
      torque = k*(rest_angle - theta);
      f_ext = manip.body(body_ind).X_joint_to_body'*[zeros(3,1);torque*manip.body(obj.joint).joint_axis];
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
        obj.rest_angle = parseParamString(model,robotnum,char(node.getAttribute('rest_length')));
      end
      if node.hasAttribute('stiffness')
        obj.k = parseParamString(model,robotnum,char(node.getAttribute('stiffness')));
      end
      
      jointNode = node.getElementsByTagName('joint').item(0);
      obj.joint = findJointInd(model,char(jointNode.getAttribute('joint')),robotnum);
    end
  end
end

