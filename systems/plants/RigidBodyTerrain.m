classdef RigidBodyTerrain < RigidBodyElement
% basic interface class for terrain
  
  methods (Abstract)
    [z,normal] = getHeight(obj,xy)
  end
  methods 
    function obj = RigidBodyTerrain()
      obj.geom = constructRigidBodyGeometry(obj);
    end
    
    function geom = getRigidBodyGeometry(obj)
      geom = obj.geom;
    end

    function geom = constructRigidBodyGeometry(obj)
      geom = [];
    end
    
    function writeWRL(obj,fptr)
    end
  end

  properties (Access = protected)
    geom % RigidBodyGeometry object
  end
  
end
