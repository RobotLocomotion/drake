classdef RigidBodyCapsule < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyCapsule(radius,len,varargin)
      % obj = RigidBodyCapsule(size) constructs a RigidBodyCapsule
      % object with the geometry-to-body transform set to identity.
      %
      % obj = RigidBodyCapsule(size,T) constructs a RigidBodyCapsule
      % object with the geometry-to-body transform T.
      % 
      % obj = RigidBodyCapsule(size,xyz,rpy) constructs a
      % RigidBodyCapsule object with the geometry-to-body transform
      % specified by the position, xyz, and Euler angles, rpy.
      %
      % @param radius - radius of the capsule
      % @param len - length of the line-segment defining the capsule.
      %   Note that because of the spherical end-caps the total length of
      %   the capsule will be len + 2*radius
      % @param T - 4x4 homogenous transform from geometry-frame to
      %   body-frame
      % @param xyz - 3-element vector specifying the position of the
      %   geometry in the body-frame
      % @param rpy - 3-element vector of Euler angles specifying the
      %   orientation of the geometry in the body-frame
      obj = obj@RigidBodyGeometry(6,varargin{:});
      sizecheck(radius,1);
      sizecheck(len,1);
      obj.radius = radius;
      obj.len = len;
    end
    
    function pts = getPoints(obj)
      % Return segment end-points
      %warning('Drake:RigidBodyGeometry:SimplifiedCollisionGeometry', ...
        %'This method returns the end points of the line segment that defines the capsule');
      cx = zeros(1,2);
      cy = zeros(1,2);
      cz = obj.len/2*[1 -1];
      
      pts = obj.T(1:end-1,:)*[cx;cy;cz;ones(1,2)];
    end

    function pts = getBoundingBoxPoints(obj)
      cx = obj.radius*[-1 1 1 -1 -1 1 1 -1];
      cy = obj.radius*[1 1 1 1 -1 -1 -1 -1];
      cz = (obj.len/2+obj.radius)*[1 1 -1 -1 -1 -1 1 1];
      
      pts = obj.T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
    end

    function geometry = serializeToLCM(obj)
      geometry = drake.lcmt_viewer_geometry_data();
      geometry.type = geometry.CAPSULE;
      geometry.string_data = '';
      geometry.num_float_data = 2;
      geometry.float_data = [obj.radius, obj.len];
      
      geometry.position = obj.T(1:3,4);
      geometry.quaternion = rotmat2quat(obj.T(1:3,1:3));
      geometry.color = [obj.c(:);1.0];
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
