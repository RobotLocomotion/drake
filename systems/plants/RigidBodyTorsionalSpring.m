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
    function [f_ext,df_ext] = computeSpatialForce(obj,manip,q,qd)
      theta = q(manip.body(obj.joint). position_num);
      torque = obj.k*(obj.rest_angle - theta);
      f_ext = sparse(6,getNumBodies(manip));
      
      if (nargout>1)
         dthetadq = zeros(1,size(q,1)); dthetadq(manip.body(obj.joint).position_num) = 1;
         dtorquedq = -obj.k*dthetadq;
         df_ext = sparse(6*getNumBodies(manip),size(q,1)+size(qd,1));
      end

      % note, because featherstone coordinates do everything about the
      % z-axis, the below is actually equivalent to
      % f_ext(:,obj.joint) = manip.body(obj.joint).X_joint_to_body'*[zeros(3,1);torque*manip.body(obj.joint).joint_axis];
      f_ext(:,obj.joint) = [zeros(2,1);torque;zeros(3,1)]; 
      if (nargout>1)
         df_ext((obj.joint-1)*6+1:obj.joint*6,1:size(q,1)) = [zeros(2,size(q,1));dtorquedq;zeros(3,size(q,1))];
         df_ext = reshape(df_ext,6,[]);
      end
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
      
      assert(numel(model.body(obj.joint).position_num)==1 && model.body(obj.joint).pitch==0,'Torsional springs are currnetly only supported for pin joints (aka, continuous and rotational joints)');
    end
  end
end

