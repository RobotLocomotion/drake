classdef RigidBodyCylinder < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyCylinder(radius,len,varargin)
      % obj = RigidBodyCylinder(radius,len) constructs a
      % RigidBodyCylinder object with the geometry-to-body transform set
      % to identity.
      %
      % obj = RigidBodyCylinder(radius,len,T) constructs a
      % RigidBodyCylinder object with the geometry-to-body transform T.
      % 
      % obj = RigidBodyCylinder(radius,len,xyz,rpy) constructs a
      % RigidBodyCylinder object with the geometry-to-body transform
      % specified by the position, xyz, and Euler angles, rpy.
      %
      % @param radius - radius of the cylinder
      % @param len - length of the cylinder
      % @param T - 4x4 homogenous transform from geometry-frame to
      %   body-frame
      % @param xyz - 3-element vector specifying the position of the
      %   geometry in the body-frame
      % @param rpy - 3-element vector of Euler angles specifying the
      %   orientation of the geometry in the body-frame
      obj = obj@RigidBodyGeometry(3,varargin{:});
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

    function pts = getBoundingBoxPoints(obj)
      % Return axis-aligned bounding-box vertices
      cx = obj.radius*[-1 1 1 -1 -1 1 1 -1];
      cy = obj.radius*[1 1 1 1 -1 -1 -1 -1];
      cz = obj.len/2*[1 1 -1 -1 -1 -1 1 1];
      
      pts = obj.T(1:3,:)*[cx;cy;cz;ones(1,8)];
    end
    
    function [x,y,z,c] = getPatchData(obj,x_axis,y_axis,view_axis)
      
      if (abs(obj.T(1:3,3)'*view_axis)>.99) 
        % just draw a circle in the viewing plane
        Tview = [x_axis, y_axis, view_axis]';
        valuecheck(svd(Tview),[1;1;1]);  % assert that it's orthonormal
        theta = 0:0.3:2*pi;
        pts = Tview'*obj.radius*[cos(theta); sin(theta); 0*theta+obj.len/2] + repmat(obj.T(1:3,4),1,length(theta));
        x = pts(1,:)';
        y = pts(2,:)';
        z = pts(3,:)';
        c = obj.c;
      else
        [x,y,z,c] = getPatchData@RigidBodyGeometry(obj,x_axis,y_axis,view_axis);
      end
      
    end 
    
    function geometry = serializeToLCM(obj)
      geometry = drake.lcmt_viewer_geometry_data();
      geometry.type = geometry.CYLINDER;
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

    function capsule = toCapsule(obj)
      % capsule = toCapsule(obj) returns a RigidBodyCapsule object with
      % the same properties (radius, length, geometry-to-body transform)
      % as obj. Note that the total length of a capsule is 2*radius
      % longer than that of a coresponding cylinder. See the constructor
      % of RigidBodyCapsule for more information.
      %
      % @param obj - RigidBodyCylinder object
      % 
      % @retvval capsule - RigidBodyCapsule object
      
      capsule = RigidBodyCapsule(obj.radius,obj.len,obj.T);
    end
    
    function h = draw(obj,model,kinsol,body_ind)
      persistent cylinder_pts; % for all cylinders
      if isempty(cylinder_pts)
        [x,y,z] = cylinder;
        cylinder_pts = [x(:)'; y(:)'; z(:)'-.5; 1+0*x(:)'];
      end
      
      pts = forwardKin(model,kinsol,body_ind,obj.T(1:3,:)*diag([obj.radius,obj.radius,obj.len,1])*cylinder_pts);
      N = size(pts,2)/2;
      % todo: specify the color
      h = surf([pts(1,1:N);pts(1,N+1:end)],[pts(2,1:N);pts(2,N+1:end)],[pts(3,1:N);pts(3,N+1:end)]);
    end
  end
  
  properties
    radius;
    len;
  end
end
