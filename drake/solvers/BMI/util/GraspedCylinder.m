classdef GraspedCylinder < GraspedGeometry
% The cylinder is parameterized as 
% R_cylinder*[r_cylinder*cos(theta);r_cylinder*sin(theta);z]+b_cylinder, where R_cylinder
% is a rotation matrix, and b_cylinder is the translation. -h_cylinder<=z<=h_cylinder
  properties(SetAccess = protected)
    R_cylinder  % A 3 x 3 rotation matrix
    b_cylinder  % A 3 x 1 translation vector
    r_cylinder  % A scalar, the radius of the cylinder
    h_cylinder  % A scalar, the half height of the cylinder
  end
  
  methods
    function obj = GraspedCylinder(quat_cylinder,b_cylinder,r_cylinder,h_cylinder)
      % @param quat_cylinder, A 4 x 1 unit quaternion, R_cylinder is the rotation
      % matrix corresponding to quat_cylinder
      obj = obj@GraspedGeometry(GraspedGeometry.CYLINDER_TYPE);
      if(any(size(quat_cylinder)~=[4,1]) || abs(norm(quat_cylinder)-1)>1e-3)
        error('quat_cylinder should be a unit quaternion');
      end
      if(any(size(b_cylinder)~=[3,1]))
        error('b_cylinder should be a 3 x 1 vector');
      end
      if(numel(r_cylinder)~=1 || r_cylinder<=0)
        error('r_cylinder should be a positive scalar');
      end
      if(numel(h_cylinder)~=1 || h_cylinder<=0)
        error('h_cylinder should be a positive scalar');
      end
      obj.R_cylinder = rotmatFromQuatBilinear(quat_cylinder*quat_cylinder');
      obj.b_cylinder = b_cylinder;
      obj.r_cylinder = r_cylinder;
      obj.h_cylinder = h_cylinder;
    end
    
    function plotGeometry(obj,use_lcmgl)
      if(use_lcmgl)
        lcmgl_cylinder = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'cylinder');
        lcmgl_cylinder.glColor3f(0,0,1);
        lcmgl_cylinder.glPushMatrix();
        lcmgl_cylinder.glTranslated(obj.b_cylinder(1),obj.b_cylinder(2),obj.b_cylinder(3));
        rotate_angle_axis = rotmat2axis(obj.R_cylinder);
        lcmgl_cylinder.glRotated(rotate_angle_axis(4)/pi*180,rotate_angle_axis(1),rotate_angle_axis(2),rotate_angle_axis(3));
        lcmgl_cylinder.cylinder([0;0;-obj.h_cylinder],obj.r_cylinder,obj.r_cylinder,2*obj.h_cylinder,20,20);
        lcmgl_cylinder.glPopMatrix();
        lcmgl_cylinder.switchBuffers();
      else
        hold on
        [x_cylinder1,y_cylinder1,z_cylinder1] = cylinder();
        z_cylinder1 = 2*z_cylinder1-1;
        A = obj.R_cylinder*diag([obj.r_cylinder,obj.r_cylinder,obj.h_cylinder]);
        b = obj.b_cylinder;
        x_cylinder = A(1,1)*x_cylinder1+A(1,2)*y_cylinder1+A(1,3)*z_cylinder1+b(1);
        y_cylinder = A(2,1)*x_cylinder1+A(2,2)*y_cylinder1+A(2,3)*z_cylinder1+b(2);
        z_cylinder = A(3,1)*x_cylinder1+A(3,2)*y_cylinder1+A(3,3)*z_cylinder1+b(3);
        h = surf(x_cylinder,y_cylinder,z_cylinder);
        alpha(0.8);
        set(h,'LineStyle','none');

        axis equal;
        grid off
        drawnow;
        hold off;
      end
    end
  end
end