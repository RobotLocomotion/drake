classdef RigidBodyForceElement
  
  properties 
    name
    direct_feedthrough_flag = false;
  end
  
  methods
    % f_ext is a (potentially sparse) matrix with manip.getNumBodies columns
    % B is (nq x nu) matrix which contributes a control-affine term 
    %      + B(q,qd)*u 
    % to the manipulator dynamics
    [f_ext,B] = computeSpatialForce(obj,manip,q,qd)
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