classdef RigidBodyConstraint
  properties(SetAccess = protected)
    type
  end
  
  properties(Constant)
    SingleTimeKinematicConstraintType = 1;
    MultipleTimeKinematicConstraintType = 2;
    QuasiStaticConstraintType = 3;
    PostureConstraintType = 4;
    MultipleTimeLinearPostureConstraint = 5;
    SingleTimeLinearPostureConstraint = 6;
  end
  
  methods
    function obj = RigidBodyConstraint(type)
      if(~isnumeric(type))
        error('Drake:Constraint:type has to be an integer');
      end
      type = floor(type);
      if(type>6||type<1)
        error('Drake:Constraint: Currently type can only be within [1,5]');
      end
      obj.type = type;
    end
  end
end