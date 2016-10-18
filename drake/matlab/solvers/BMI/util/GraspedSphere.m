classdef GraspedSphere < GraspedGeometry
  properties(SetAccess = protected)
    sphere_radius
    sphere_center
  end
  
  methods
    function obj = GraspedSphere(sphere_radius,sphere_center)
      obj = obj@GraspedGeometry(GraspedGeometry.SPHERE_TYPE);
      if(numel(sphere_radius) ~= 1 || sphere_radius<=0)
        error('sphere_radius should be positive scalar');
      end
      if(any(size(sphere_center)~= [3,1]))
        error('sphere_center should be a 3 x 1 vector');
      end
      obj.sphere_radius = sphere_radius;
      obj.sphere_center = sphere_center;
    end
    
    function obj = plotGeometry(obj,use_lcmgl)
      if(use_lcmgl)
        lcmgl_sphere = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'sphere');
        lcmgl_sphere.glColor4f(0,0,1,0.4);
        lcmgl_sphere.sphere(obj.sphere_center,obj.sphere_radius,20,20);
        lcmgl_sphere.switchBuffers();
      else
        hold on;
        [x_sphere,y_sphere,z_sphere] = sphere(40);
        x_sphere = obj.sphere_radius*x_sphere+obj.sphere_center(1);
        y_sphere = obj.sphere_radius*y_sphere+obj.sphere_center(2);
        z_sphere = obj.sphere_radius*z_sphere+obj.sphere_center(3);
        h = surf(x_sphere,y_sphere,z_sphere);
        set(h,'FaceColor',[0,0,1]);
        set(h,'LineStyle','none');

        axis equal
        grid off
        drawnow;
        hold off;
      end
    end
  end
end
