classdef RigidBodyCylinder < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyCylinder(radius,len)
      sizecheck(radius,1);
      sizecheck(len,1);
      obj.radius = radius;
      obj.len = len;
    end
    
    function pts = getPoints(obj)
      % treat it as a box, for collisions
      warning('Drake:RigidBodyGeometry:SimplifiedCollisionGeometry','for efficiency, cylinder geometry will be treated like a box for 3D collisions');
      cx = obj.radius*[-1 1 1 -1 -1 1 1 -1];
      cy = obj.radius*[1 1 1 1 -1 -1 -1 -1];
      cz = obj.len/2*[1 1 -1 -1 -1 -1 1 1];
      
      pts = obj.T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
    end

    function shape = serializeToLCM(obj)
      shape = drake.lcmt_viewer_geometry_data();
      shape.type = shape.CYLINDER;
      shape.string_data = '';
      shape.num_float_data = 2;
      shape.float_data = [obj.radius, obj.len];
      
      shape.position = obj.T(1:3,4);
      shape.quaternion = rotmat2quat(obj.T(1:3,1:3));
      shape.color = [obj.c(:);1.0];
    end

  end
  
  properties
    radius;
    len;
  end
end