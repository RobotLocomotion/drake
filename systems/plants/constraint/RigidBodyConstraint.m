classdef RigidBodyConstraint
  properties
    type
  end
  
  properties(Constant)
    SingleTimeKinematicConstraintType = 1;
    MultipleTimeKinematicConstraintType = 2;
    QuasiStaticConstraintType = 3;
    PostureConstraintType = 4;
    MultipleTimeLinearPostureConstraint = 5;
  end
  
  methods
    function obj = RigidBodyConstraint(type)
      if(~isnumeric(type))
        error('Drake:Constraint:type has to be an integer');
      end
      type = floor(type);
      if(type>5||type<1)
        error('Drake:Constraint: Currently type can only be within [1,5]');
      end
      obj.type = type;
    end
  end
end