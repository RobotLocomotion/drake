classdef Constraint
  properties
    type
  end
  
  properties(Constant)
    KinematicConstraintType = 1;
    QuasiStaticConstraintType = 2;
    PostureConstraintType = 3;
  end
  
  methods
    function obj = Constraint(type)
      if(~isnumeric(type))
        error('Drake:Constraint:type has to be an integer');
      end
      type = floor(type);
      if(type>3||type<1)
        error('Drake:Constraint: Currently type can only be within [1,3]');
      end
      obj.type = type;
    end
  end
end