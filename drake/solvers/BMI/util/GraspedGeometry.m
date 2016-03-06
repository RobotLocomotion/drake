classdef GraspedGeometry
  properties(SetAccess = protected)
    type
  end
  
  properties(Constant)
    POLYHEDRON_TYPE = 1;
    SPHERE_TYPE = 2;
    ELLIPSOID_TYPE = 3;
    CYLINDER_TYPE = 4;
  end
  
  methods
    function obj = GraspedGeometry(type)
      obj.type = type;
    end
  end
  
  methods(Abstract)
    plotGeometry(obj,use_lcmgl);
  end
end