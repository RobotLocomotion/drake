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
      obj.drake_shape_id = 5;
    end
    
    function points = getPoints(obj)
      points = obj.points;
    end
    
    function msg = serializeToLCM(obj)
      warning('Drake:RigidBodyMeshPoints:NoOBJVisualization','OBJ visualization is not implemented yet for RigidBodyMeshPoints (but it shouldn''t be too hard)');
      msg = drake.lcmt_viewer_geometry_data();
      msg.type = msg.SPHERE;
      msg.string_data = '';
      msg.num_float_data = 1;
      msg.float_data = 0;
    end
    
    function writeWRLShape(obj,fp,td)
      warning('Drake:RigidBodyMeshPoints:NoWRLVisualization','WRL visualization is not implemented yet for RigidBodyMeshPoints (but it would be easy)');
    end
    
  end
  
  properties
    points
  end
  
end
