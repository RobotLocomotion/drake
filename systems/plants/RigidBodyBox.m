classdef RigidBodyBox < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyBox(size)
      obj = obj@RigidBodyGeometry(1);
      sizecheck(size,3);
      obj.size = size(:);
    end
    
    function pts = getPoints(obj)
      cx = obj.size(1)/2*[-1 1 1 -1 -1 1 1 -1];
      cy = obj.size(2)/2*[1 1 1 1 -1 -1 -1 -1];
      cz = obj.size(3)/2*[1 1 -1 -1 -1 -1 1 1];
      
      pts = obj.T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
    end

    function pts = getBoundingBoxPoints(obj)
      pts = getPoints(obj);
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
    
    function writeWRLShape(obj,fp,td)
      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end

      tabprintf(fp,'Transform {\n'); td=td+1;
      tabprintf(fp,'translation %f %f %f\n',obj.T(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(obj.T(1:3,1:3)));

      tabprintf(fp,'children Shape {\n'); td=td+1;
      tabprintf(fp,'geometry Box { size %f %f %f }\n',obj.size);
      tabprintf(fp,'appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
      td=td-1; tabprintf(fp,'}\n');  % end Shape {
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end

    function has_terrain_contact_points = hasTerrainContactPoints(obj)
      has_terrain_contact_points = true;
    end
  end
  
  properties
    size  % 3x1 vector with x size, y size, z size
  end
  
end
