classdef RigidBodyFlatTerrainNoGeometry < RigidBodyTerrain
%  This class provides an implementation of RigidBodyTerrain with z=0
%  everywhere. In contrast to `RigidBodyFlatTerrain`, this class does
%  not add `RigidBodyGeometry` objects to the manipulator. It is
%  therefore useful for testing the plumbing for non-flat terrain types.
  
  methods 
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      z = repmat(0,1,n);
      normal = repmat([0;0;1],1,n);
    end
  end
end
