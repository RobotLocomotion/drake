classdef GraspedEllipsoid < GraspedGeometry
  properties(SetAccess = protected)
    % The ellipsoid is A_ellipsoid * r + b_ellipsoid, |r| <= 1
    A_ellipsoid % A 3 x 3 matrix
    b_ellipsoid % A 3 x 1 vector
    min_eig_A  % The minimum singular value of A_ellipsoid
  end
  
  methods
    function obj = GraspedEllipsoid(A_ellipsoid, b_ellipsoid)
      obj = obj@GraspedGeometry(GraspedGeometry.ELLIPSOID_TYPE);
      if(any(size(A_ellipsoid) ~= [3,3])||rank(A_ellipsoid)<3)
        error('A_ellipsoid should be 3 x 3 full rank matrix');
      end
      if(any(size(b_ellipsoid) ~= [3,1]))
        error('b_ellipsoid should be 3 x 1 vector');
      end
      D_A_ellipsoid = eig(A_ellipsoid'*A_ellipsoid);
      obj.min_eig_A = sqrt(min(D_A_ellipsoid));
      obj.A_ellipsoid = A_ellipsoid;
      obj.b_ellipsoid = b_ellipsoid;
    end
    
    function plotGeometry(obj,use_lcmgl)
      if(use_lcmgl)
        lcmgl_ellipsoid = drawEllipsoid(obj.A_ellipsoid,obj.b_ellipsoid,'ellipsoid');
        lcmgl_ellipsoid.switchBuffers();
      else
        hold on;
        [x_sphere,y_sphere,z_sphere] = sphere(20);
        x_ellipsoid = obj.A_ellipsoid(1,1)*x_sphere+obj.A_ellipsoid(1,2)*y_sphere+obj.A_ellipsoid(1,3)*z_sphere+obj.b_ellipsoid(1);
        y_ellipsoid = obj.A_ellipsoid(2,1)*x_sphere+obj.A_ellipsoid(2,2)*y_sphere+obj.A_ellipsoid(2,3)*z_sphere+obj.b_ellipsoid(2);
        z_ellipsoid = obj.A_ellipsoid(3,1)*x_sphere+obj.A_ellipsoid(3,2)*y_sphere+obj.A_ellipsoid(3,3)*z_sphere+obj.b_ellipsoid(3);
        h = surf(x_ellipsoid,y_ellipsoid,z_ellipsoid);
        alpha(0.5);
        set(h,'FaceColor',[0,0,1],'LineStyle','none');
      
        axis equal
        grid off
        drawnow;
        hold off;
      end
    end
  end
end