classdef RigidBodyForceElement
  
  properties
    name
  end
  
  methods
    % f_ext is a (potentially sparse) matrix with manip.getNumDOF columns
    f_ext = computeSpatialForce(obj,manip,q,qd)
  end
  
  methods (Static=true)
    function f = cartesianForceToSpatialForce(point,force)  % from Fpt in featherstone v2
      if length(force)==3
        f = [ cross(point,force,1); force ];
      else
        f = [ point(1,:).*force(2,:) - point(2,:).*force(1,:); force ];
      end
    end
  end
  
end