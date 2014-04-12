classdef RigidBodyMeshPts < RigidBodyMesh
  
  methods
    function obj = RigidBodyMeshPts(pts)
      obj = obj@RigidBodyMesh('');
      obj.pts = pts;
    end
    
    function pts = getPoints(obj)
      pts = obj.pts;
    end
    
    function msg = serializeToLCM(obj)
      error('not implemented yet');
    end
  end
  
  properties
    pts
  end
  
end
