classdef RigidBodyFlatTerrainNoGeometry < RigidBodyTerrain
%  This class provides an implementation of RigidBodyTerrain with z=0
%  everywhere
  
  methods 
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      z = repmat(0,1,n);
      normal = repmat([0;0;1],1,n);
    end
  end
end
