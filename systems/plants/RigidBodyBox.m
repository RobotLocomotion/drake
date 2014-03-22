classdef RigidBodyBox < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyBox(size)
      sizecheck(size,3);
      obj.size = size(:);
    end
    
    function pts = getPoints(obj)
      cx = obj.size(1)/2*[-1 1 1 -1 -1 1 1 -1];
      cy = obj.size(2)/2*[1 1 1 1 -1 -1 -1 -1];
      cz = obj.size(3)/2*[1 1 -1 -1 -1 -1 1 1];
      
      pts = obj.T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
    end
    
    function shape = serializeToLCM(obj)
      shape = drake.lcmt_viewer_geometry_data();
      shape.type = shape.BOX;
      shape.string_data = '';
      shape.num_float_data = 3;
      shape.float_data = obj.size;
      
      shape.position = obj.T(1:3,4);
      shape.quaternion = rotmat2quat(obj.T(1:3,1:3));
      shape.color = [obj.c(:);1.0];
    end

  end
  
  properties
    size  % 3x1 vector with x size, y size, z size
  end
  
end