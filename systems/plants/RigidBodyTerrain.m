classdef RigidBodyTerrain < RigidBodyElement
% basic interface class for terrain
  
  methods (Abstract)
    [z,normal] = getHeight(obj,xy)
  end
  methods 
    function obj = RigidBodyTerrain()
    end
    
    function geom = getCollisionGeometry(obj)
      geom = [];
    end

    function geom = getVisualGeometry(obj)
      geom = [];
    end
  end

end
