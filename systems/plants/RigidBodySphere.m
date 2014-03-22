classdef RigidBodySphere < RigidBodyGeometry
  
  methods 
    function obj = RigidBodySphere(radius)
      sizecheck(radius,1);
      obj.radius = radius;
    end
    
    function pts = getPoints(obj)
      if (obj.radius~=0)
        warning('Drake:RigidBodySphere:SimplifiedCollisionGeometry','for efficiency, 3D sphere geometry will be treated like a point (at the center of the sphere)');
      end
      pts = obj.T(1:3,4);
    end
    
    function pts = getPlanarPoints(obj,x_axis,y_axis,view_axis)
      Tview = [x_axis, y_axis, view_axis]';
      valuecheck(svd(Tview),[1;1;1]);  % assert that it's orthonormal
      
      if (obj.radius==0)
        pts = s.T(1:3,4);
      else
        % just draw a circle in the viewing plane
        theta = 0:0.1:2*pi;
        pts = Tview'*obj.radius*[cos(theta); sin(theta); 0*theta] + repmat(obj.T(1:3,4),1,length(theta));
      end
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

  end
  
  properties
    radius;
  end
end