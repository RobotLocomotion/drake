classdef RigidBodyJointTransmission < RigidBodyElement
% A mechanical transmission, such that joint1 = multiplier*joint2+offset
  properties
    joint1_name
    joint2_name
    multiplier
    offset
    robotnum
    constraint_id
  end
  
  methods
    function [obj,model] = updateConstraints(obj,model)
      link1 = model.findJointId(obj.joint1_name,obj.robotnum);
      link2 = model.findJointId(obj.joint2_name,obj.robotnum);
      joint1_pos_index = model.getBody(link1).position_num;
      joint2_pos_index = model.getBody(link2).position_num;
      A = zeros(1,model.getNumPositions());
      A(joint1_pos_index) = 1;
      A(joint2_pos_index) = -obj.multiplier;
      fcn = drakeFunction.Affine(A,-obj.offset);
      cnstr = DrakeFunctionConstraint(0,0,fcn);
      cnstr.grad_level = 2;
      cnstr.setName({sprintf('%s mimics %s through joint transmission',obj.joint1_name,obj.joint2_name)});
      
      if(isempty(obj.constraint_id))
        [model,obj.constraint_id] = addPositionEqualityConstraint(model,cnstr);
      else
        model = updatePositionEqualityConstraint(model,obj.constraint_id,cnstr);
      end
    end
  end
end