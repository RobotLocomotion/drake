classdef RigidBodyFlatTerrain < RigidBodyTerrain
%  This class provides an implementation of RigidBodyTerrain with z=0
%  everywhere
  
  methods 
    function obj = RigidBodyFlatTerrain(varargin)
      % Construct a flat terrain map at a given z height (default 0)
      p = inputParser();
      p.addOptional('z', 0, @isscalar);
      p.parse(varargin{:});
      obj.z = p.Results.z;
      obj.geom = constructRigidBodyGeometry(obj);
    end
    
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      z = repmat(obj.z,1,n);
      normal = repmat([0;0;1],1,n);
    end

    function geom = getCollisionGeometry(obj)
      geom = obj.geom;
    end

    function geom = getVisualGeometry(obj)
      geom = obj.geom;
    end

    function obj = setGeometryColor(obj, color)
      geom = obj.getVisualGeometry();
      geom.c = reshape(color, 3, 1);
      obj.geom = geom;
    end
    
    function geom = constructRigidBodyGeometry(obj)
      box_width = 1000;
      box_depth = 10;
      geom = RigidBodyBox([box_width;box_width;box_depth]);
      geom.T(3,4) = obj.z - box_depth/2;
      geom.c = hex2dec({'ee','cb','ad'})'/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/);
      geom.name = 'terrain';
%      geom.c = hex2dec({'cd','af','95'})'/256;
    end
  end
  
  properties
    geom;
    z;
  end
end
