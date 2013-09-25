classdef Constraint
  properties
    type
  end
  
  properties(Constant)
    SingleTimeKinematicConstraintType = 1;
    MultipleTimeKinematicConstraintType = 2;
    QuasiStaticConstraintType = 3;
    PostureConstraintType = 4;
  end
  
  methods
    function obj = Constraint(type)
      if(~isnumeric(type))
        error('Drake:Constraint','Type has to be an integer');
      end
      type = floor(type);
      if(type>4||type<1)
        error('Drake:Constraint','Currently type can only be within [1,3]');
      end
      obj.type = type;
    end
  end
end