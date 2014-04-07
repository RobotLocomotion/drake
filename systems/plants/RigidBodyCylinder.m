classdef RigidBodyCylinder < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyCylinder(radius,len)
      obj = obj@RigidBodyGeometry(3);
      sizecheck(radius,1);
      sizecheck(len,1);
      obj.radius = radius;
      obj.len = len;
    end
    
    function pts = getPoints(obj)
      warning('Drake:RigidBodyGeometry:SimplifiedCollisionGeometry', ...
              'This method returns the vertices of the cylinder''s bounding-box.');
      pts = getBoundingBoxPoints(obj);
    end

    function pts = getPoints(obj)
      % Return axis-aligned bounding-box vertices
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

    function writeWRLShape(obj,fp,td)
      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end

      % default axis for cylinder in urdf is the z-axis, but
      % the default in vrml is the y-axis.
      T = obj.T*[1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1];
      tabprintf(fp,'Transform {\n'); td=td+1;
      tabprintf(fp,'translation %f %f %f\n',T(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(T(1:3,1:3)));

      tabprintf(fp,'children Shape {\n'); td=td+1;
      tabprintf(fp,'geometry Cylinder { height %f\t radius %f }\n',obj.len,obj.radius);
      tabprintf(fp,'appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
      td=td-1; tabprintf(fp,'}\n');  % end Shape {
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end
  end
  
  properties
    radius;
    len;
  end
end
