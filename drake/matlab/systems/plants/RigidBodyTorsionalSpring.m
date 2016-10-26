classdef RigidBodyTorsionalSpring < RigidBodyForceElement
%  Adds torsionals linear springs, with 
%      torque = k*(rest_angle - current_angle),
%  to rigid body manipulators.  
  
  properties
    parent_body
    child_body 
    rest_angle=0
    k=0
  end
  
  methods
    function [f_ext,df_ext] = computeSpatialForce(obj,manip,q,qd)
      theta = q(manip.body(obj.child_body).position_num);
      torque = obj.k*(obj.rest_angle - theta);
      f_ext = q(1)*sparse(6,getNumBodies(manip));
      
      if (nargout>1)
         dthetadq = zeros(1,size(q,1)); dthetadq(manip.body(obj.child_body).position_num) = 1;
         dtorquedq = -obj.k*dthetadq;
         df_ext = sparse(6*getNumBodies(manip),size(q,1)+size(qd,1));
      end

      wrench_on_child_in_child_frame = [manip.body(obj.child_body).joint_axis * torque; zeros(3, 1)];
      f_ext(:,obj.child_body) = wrench_on_child_in_child_frame;

      if obj.parent_body ~= 0 % don't apply force to world body
        T_parent_to_child_joint_predecessor = homogTransInv(manip.body(obj.child_body).Ttree);
        AdT_parent_to_child_joint_predecessor = transformAdjoint(T_parent_to_child_joint_predecessor);
        f_ext(:,obj.parent_body) = -AdT_parent_to_child_joint_predecessor' * wrench_on_child_in_child_frame; 
      end
      if (nargout>1)
        df_ext((obj.child_body-1)*6+1:obj.child_body*6,1:size(q,1)) = [manip.body(obj.child_body).joint_axis * dtorquedq; zeros(3,size(q,1))];
         if obj.parent_body ~= 0
           df_ext((obj.parent_body-1)*6+1:obj.parent_body*6,1:size(q,1)) = -AdT_parent_to_child_joint_predecessor' * dwrench_on_child_in_child_joint_frame;
         end
      end
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.child_body = map_from_old_to_new(obj.child_body);
      obj.parent_body = map_from_old_to_new(obj.parent_body);
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.child_body == body_ind)
        error('removed joint with a torsional spring.  need to handle this case');
      end
    end
    function [T,U] = energy(obj,manip,q,qd)
      T=0;
      theta = q(manip.body(obj.child_body).position_num);
      U = .5*obj.k*(obj.rest_angle - theta)'*(obj.rest_angle - theta);
    end
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,name,robotnum,node,options)
      obj = RigidBodyTorsionalSpring();
      obj.name = name;
      
      if node.hasAttribute('rest_angle')
        obj.rest_angle = parseParamString(model,robotnum,char(node.getAttribute('rest_angle')));
      end
      if node.hasAttribute('stiffness')
        obj.k = parseParamString(model,robotnum,char(node.getAttribute('stiffness')));
      end
      
      jointNode = node.getElementsByTagName('joint').item(0);
      obj.child_body = findJointId(model,char(jointNode.getAttribute('name')),robotnum);
      obj.parent_body = model.body(obj.child_body).parent;
      assert(numel(model.body(obj.child_body).position_num)==1 && model.body(obj.child_body).pitch==0,'Torsional springs are currnetly only supported for pin joints (aka, continuous and rotational joints)');
    end
  end
end

