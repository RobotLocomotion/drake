classdef RigidBodyForceElement
  
  properties
    name
  end
  
  methods
    % f_ext is a (potentially sparse) matrix with manip.getNumDOF columns
    f_ext = computeSpatialForce(obj,manip,q,qd)
  end
  
  methods (Static=true)
    function f = cartesianForceToSpatialForce(varargin);
      error('this method is deprecated.  use RigidBodyManipulator.cartesianForceToSpatialForce instead');
    end
  end
  
end