classdef RigidBodyMeshPoints < RigidBodyMesh
  % RigidBodyMeshPoints   Represents the convex hull of a set of points
  % This class allows for the programatic creation of geometries
  % consisting of the convex hull of a set of points. Visualization is
  % not yet supported for this class.
  %
  % RigidBodyMeshPoints properties:
  %   points - 3 x m array in which the i-th column specifies the
  %            location of the i-th point in body-frame.
  %
  % See also RigidBodyGeometry, RigidBodyMesh
  methods
    function obj = RigidBodyMeshPoints(points)
      obj = obj@RigidBodyMesh('');
      obj.points = points;
    end
    
    function points = getPoints(obj)
      points = obj.points;
    end
    
    function msg = serializeToLCM(obj)
      error('not implemented yet');
    end
  end
  
  properties
    points
  end
  
end
