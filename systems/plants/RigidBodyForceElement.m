classdef RigidBodyForceElement
  
  properties
    name
  end
  
  methods
    % f_ext is a (potentially sparse) matrix with manip.getNumBodies columns
    f_ext = computeSpatialForce(obj,manip,q,qd)
  end
  
  methods
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      % intentionally do nothing. overload if necessary
    end
  end
  
  methods (Static=true)
    function f = cartesianForceToSpatialForce(varargin);
      error('this method is deprecated.  use RigidBodyManipulator.cartesianForceToSpatialForce instead');
    end
  end
  
end