classdef RigidBodyTerrain < RigidBodyElement
% basic interface class for terrain
  
  methods (Abstract)
    [z,normal] = getHeight(obj,xy)
  end
  methods 
    function obj = RigidBodyTerrain()
    end
    
    function geom = getRigidBodyContactGeometry(obj)
      geom = [];
    end

    function geom = getRigidBodyShapeGeometry(obj)
      geom = [];
    end
  end

end
