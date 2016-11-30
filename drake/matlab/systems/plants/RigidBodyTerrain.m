classdef RigidBodyTerrain < RigidBodyElement
% basic interface class for terrain
  
  methods (Abstract)
    [z,normal] = getHeight(obj,xy)
  end
  methods 
    function obj = RigidBodyTerrain()
    end
    
    function varargout = getRigidBodyContactGeometry(varargin)
      errorDeprecatedFunction('getCollisionGeometry');
    end
    
    function geom = getCollisionGeometry(obj)
      geom = [];
    end
    
    function varargout = getRigidBodyShapeGeometry(varargin)
      errorDeprecatedFunction('getVisualGeometry');
    end

    function geom = getVisualGeometry(obj)
      geom = [];
    end
  end

end
