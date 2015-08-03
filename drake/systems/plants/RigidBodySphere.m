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
    
    function geometry = serializeToLCM(obj)
      geometry = drake.lcmt_viewer_geometry_data();
      geometry.type = geometry.SPHERE;
      geometry.string_data = '';
      geometry.num_float_data = 1;
      geometry.float_data = obj.radius;

      geometry.position = obj.T(1:3,4);
      geometry.quaternion = rotmat2quat(obj.T(1:3,1:3));
      geometry.color = [obj.c(:);1.0];
    end

    function writeWRLShape(obj,fp,td)
      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end

      tabprintf(fp,'Transform {\n'); td=td+1;
      tabprintf(fp,'translation %f %f %f\n',obj.T(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(obj.T(1:3,1:3)));

      tabprintf(fp,'children Shape {\n'); td=td+1;
      tabprintf(fp,'geometry Sphere { radius %f }\n',obj.radius);
      tabprintf(fp,'appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
      td=td-1; tabprintf(fp,'}\n');  % end Shape {
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end

    function pts = getTerrainContactPoints(obj)
      % pts = getTerrainContactPoints(obj) returns the terrain contact points
      % of this object. A zero-radius sphere has a single terrain contact point
      % located at its origin. Non-zeros-radius spheres have no terrain contact
      % points.
      %
      % For a general description of terrain contact points see 
      % <a href="matlab:help RigidBodyGeometry/getTerrainContactPoints">RigidBodyGeometry/getTerrainContactPoints</a>
      %
      % @param  obj - RigidBodySphere object
      % @retval pts - 3xm array of points on this geometry (in link frame) that
      %               can collide with the world.
      if obj.radius < 1e-6
        pts = getPoints(obj);
      else
        pts=[];
      end
    end
  
    function h = draw(obj,model,kinsol,body_ind)
      persistent sphere_pts; % for all spheres
      if isempty(sphere_pts)
        [x,y,z] = sphere;
        sphere_pts = [x(:)'; y(:)'; z(:)'; 1+0*x(:)'];
      end
      
      pts = forwardKin(model,kinsol,body_ind,obj.T(1:3,:)*obj.radius*sphere_pts);
      N = size(pts,2)/2;
      % todo: specify the color
      h = surf([pts(1,1:N);pts(1,N+1:end)],[pts(2,1:N);pts(2,N+1:end)],[pts(3,1:N);pts(3,N+1:end)]);
    end    
  end
  properties
    radius;
  end
end
