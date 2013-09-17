classdef Constraint
  properties
    type
  end
  
  properties(Constant)
    KinematicConstraint = 1;
    QuasiStaticConstraint = 2;
    PostureConstraint = 3;
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