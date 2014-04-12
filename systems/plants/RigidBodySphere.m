classdef RigidBodySphere < RigidBodyGeometry
  
  methods 
    function obj = RigidBodySphere(radius,varargin)
      % obj = RigidBodySphere(radius) constructs a RigidBodySphere
      % object with the geometry-to-body transform set to identity.
      %
      % obj = RigidBodySphere(radius,T) constructs a RigidBodySphere
      % object with the geometry-to-body transform T.
      % 
      % obj = RigidBodySphere(radius,xyz,rpy) constructs a
      % RigidBodySphere object with the geometry-to-body transform
      % specified by the position, xyz, and Euler angles, rpy.
      %
      % @param radius - radius of the sphere
      % @param T - 4x4 homogenous transform from geometry-frame to
      %   body-frame
      % @param xyz - 3-element vector specifying the position of the
      %   geometry in the body-frame
      % @param rpy - 3-element vector of Euler angles specifying the
      %   orientation of the geometry in the body-frame
      obj = obj@RigidBodyGeometry(2,varargin{:});
      sizecheck(radius,1);
      obj.radius = radius;
    end
    
    function pts = getPoints(obj)
      if (obj.radius~=0)
        warning('Drake:RigidBodySphere:SimplifiedCollisionGeometry','for efficiency, 3D sphere geometry will be treated like a point (at the center of the sphere)');
      end
      pts = obj.T(1:3,4);
    end

    function pts = getBoundingBoxPoints(obj)
      % Return axis-aligned bounding-box vertices
      cx = obj.radius*[-1 1 1 -1 -1 1 1 -1];
      cy = obj.radius*[1 1 1 1 -1 -1 -1 -1];
      cz = obj.radius*[1 1 -1 -1 -1 -1 1 1];
      
      pts = obj.T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
    end
    
    function [x,y,z,c] = getPatchData(obj,x_axis,y_axis,view_axis)
      Tview = [x_axis, y_axis, view_axis]';
      valuecheck(svd(Tview),[1;1;1]);  % assert that it's orthonormal
      
      if (obj.radius==0)
        pts = s.T(1:3,4);
      else
        % just draw a circle in the viewing plane
        theta = 0:0.1:2*pi;
        pts = Tview'*obj.radius*[cos(theta); sin(theta); 0*theta] + repmat(obj.T(1:3,4),1,length(theta));
      end
      x = pts(1,:)';
      y = pts(2,:)';
      z = pts(3,:)';
      c = obj.c;
    end
    
    function shape = serializeToLCM(obj)
      shape = drake.lcmt_viewer_geometry_data();
      shape.type = shape.SPHERE;
      shape.string_data = '';
      shape.num_float_data = 1;
      shape.float_data = obj.radius;

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
      tabprintf(fp,'geometry Sphere { radius %f }]n',obj.radius);
      tabprintf(fp,'appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
      td=td-1; tabprintf(fp,'}\n');  % end Shape {
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end

    function pts = getTerrainContactPoints(obj)
      % pts = getTerrainContactPoints(obj)
      %
      % @param  obj - RigidBodySphere object
      % @retval pts - 3xm array of points on this geometry (in link frame) that
      %               can collide with the world.
      if obj.radius == 0
        pts = getPoints(obj);
      end
    end
  
  end
  properties
    radius;
  end
end
