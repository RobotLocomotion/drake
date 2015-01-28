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
      f_ext = sparse(6,getNumBodies(manip));
      
      if (nargout>1)
         dthetadq = zeros(1,size(q,1)); dthetadq(manip.body(obj.child_body).position_num) = 1;
         dtorquedq = -obj.k*dthetadq;
         df_ext = sparse(6*getNumBodies(manip),size(q,1)+size(qd,1));
      end

      % note, because featherstone coordinates do everything about the
      % z-axis, the below is actually equivalent to
      % f_ext(:,obj.child_body) = manip.body(obj.child_body).X_joint_to_body'*[zeros(3,1);torque*manip.body(obj.child_body).joint_axis];
      f_ext(:,obj.child_body) = [zeros(2,1);torque;zeros(3,1)]; 
      if obj.parent_body ~= 0 % don't apply force to world body
        f_ext(:,obj.parent_body) = -[zeros(2,1);torque;zeros(3,1)]; 
      end
      if (nargout>1)
         df_ext((obj.child_body-1)*6+1:obj.child_body*6,1:size(q,1)) = [zeros(2,size(q,1));dtorquedq;zeros(3,size(q,1))];
         if obj.parent_body ~= 0
           df_ext((obj.parent_body-1)*6+1:obj.parent_body*6,1:size(q,1)) = -[zeros(2,size(q,1));dtorquedq;zeros(3,size(q,1))];
         end
         df_ext = reshape(df_ext,6,[]);
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
      obj.child_body = findJointId(model,char(jointNode.getAttribute('name')),robotnum);
      obj.parent_body = model.body(obj.child_body).parent;
      assert(numel(model.body(obj.child_body).position_num)==1 && model.body(obj.child_body).pitch==0,'Torsional springs are currnetly only supported for pin joints (aka, continuous and rotational joints)');
    end
  end
end

