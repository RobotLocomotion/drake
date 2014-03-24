classdef RigidBodyForceElement < RigidBodyElement
  
  properties 
    name
    direct_feedthrough_flag = false;
    input_num = [];
    input_limits = [];
  end
  
  methods
    % f_ext is a (potentially sparse) matrix with manip.getNumBodies columns
    % B is (nq x nu) matrix which contributes a control-affine term 
    %      + B(q,qd)*u 
    % to the manipulator dynamics
    [f_ext,B] = computeSpatialForce(obj,manip,q,qd)
  end
  
  methods
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      error('probably need to implement this (see changeRootLink)');
    end
  end
  
  methods (Static=true)
    function f = cartesianForceToSpatialForce(varargin);
      error('this method is deprecated.  use RigidBodyManipulator.cartesianForceToSpatialForce instead');
    end
  end
  
end