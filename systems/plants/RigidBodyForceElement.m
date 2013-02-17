classdef RigidBodyForceElement
  
  properties
    name
  end
  
  methods
    % f_ext is a (potentially sparse) matrix with manip.getNumDOF columns
    f_ext = computeSpatialForce(obj,manip,q,qd)
  end
  
  methods (Static=true)
    function f = cartesianForceToSpatialForce(point,force)
      f = [ cross(point,force,1); force ];
    end
  end
  
end