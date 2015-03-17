classdef Cartesian2CylindricalTransform < drakeFunction.DrakeFunction
  % Given a cartesian coordinate frame, transform its coordinates to a
  % cylindrical coordinates. Please refer to
  % drake/doc/doc_cartesian2cylindrical.pdf for details on the coordinates.
  properties(SetAccess = protected)
    cylinder_axis % A 3 x 1 vector, the axis of the cylinder in the Cartesian coordinate
    cylinder_x_dir % A 3 x 1 vector, the cylinder x direction (the direction that theta=0 points to) in the Cartesian coordinate
    cylinder_origin % A 3 x 1 vector, the origin (height = 0) point of the cylinder in the Cartesian coordinate
    T_cylinder % This is a homogeneous transformation matrix, T_cylinder transforms a cylinder with [0;0;1] being its axis, and the angle theta measured with respect to the x-axis
  end
  
  methods
    function obj = Cartesian2CylindricalTransform(cylinder_axis,cylinder_x_dir,cylinder_origin)
      input_frame = drakeFunction.frames.realCoordinateSpace(6);
      output_frame = drakeFunction.frames.realCoordinateSpace(6);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      sizecheck(cylinder_axis,[3,1]);
      sizecheck(cylinder_origin,[3,1]);
      sizecheck(cylinder_x_dir,[3,1]);
      axis_norm = norm(cylinder_axis);
      if(axis_norm<eps)
        error('cylinder_axis should be a non-zero vector');
      end
      x_dir_norm = norm(cylinder_x_dir);
      if(x_dir_norm<eps)
        error('cylinder_x_dir should be a non-zero vector');
      end
      
      obj.cylinder_axis = cylinder_axis/axis_norm;
      obj.cylinder_x_dir = cylinder_x_dir/x_dir_norm;
      obj.cylinder_origin  = cylinder_origin;
      if(abs(obj.cylinder_axis'*obj.cylinder_x_dir)>1e-10)
        error('cylinder_axis and cylinder_x_dir should be perpendicular to each other');
      end
      R_cylinder = [obj.cylinder_x_dir cross(obj.cylinder_axis,obj.cylinder_x_dir) obj.cylinder_axis];
      % compute the homogeneous transformation that translate by
      % cylinder_origin and rotate [0;0;1] to cylinder_axis
      obj.T_cylinder = [R_cylinder obj.cylinder_origin;0 0 0 1];
    end
    
    function [x_cylinder,J] = eval(obj,x_cartesian)
      % x_cylinder = [radius;theta;height;roll;pitch;yaw]
      % we attach a cartesian frame [x_tangent;y_normal;z_tangent] in R^3x3 at the tip of the
      % radius, where x_tangent is aligned with cross(radius,height),
      % y_norm is along the radius direction, and z_tangent =
      % cross(x_tangent,y_normal). roll;pitch;yaw are the Euler angles that
      % would rotate cartesian frame [x_tangent;y_normal;z_tangent] to
      % align with the cartesian frame x_cartesian
      dx_pos_cylinder = [inv(obj.T_cylinder(1:3,1:3)) zeros(3,3)];
      x_pos_cylinder = dx_pos_cylinder(:,1:3)*(x_cartesian(1:3)-obj.T_cylinder(1:3,4));
      radius = sqrt(x_pos_cylinder(1)^2+x_pos_cylinder(2)^2);
      dradius_dx_pos_cylinder = [x_pos_cylinder(1:2)' 0]/radius;
      theta = atan2(x_pos_cylinder(2),x_pos_cylinder(1));
      dtheta_dx_pos_cylinder = [datan2(x_pos_cylinder(1),x_pos_cylinder(2)) 0];
      height = x_pos_cylinder(3);
      dheight_dx_pos_cylinder = [0 0 1];
      x_cylinder = zeros(6,1);
      x_cylinder(1:3) = [radius;theta;height];
      J = zeros(6,6);
      J(1:3,:) = [dradius_dx_pos_cylinder;dtheta_dx_pos_cylinder;dheight_dx_pos_cylinder]*dx_pos_cylinder;
      [x_rotmat,dx_rotmat] = rpy2rotmat(x_cartesian(4:6));
      [R_cylinder2tangent,dR_cylinder2tangent] = rotz(theta-pi/2);
      x_rotmat_cylinder = R_cylinder2tangent*dx_pos_cylinder(:,1:3)*x_rotmat;
      dx_rotmat_cylinder = matGradMult(dR_cylinder2tangent(:)*dtheta_dx_pos_cylinder*dx_pos_cylinder,dx_pos_cylinder(:,1:3)*x_rotmat)+...
        reshape(R_cylinder2tangent*dx_pos_cylinder(:,1:3)*reshape([zeros(9,3) dx_rotmat],3,[]),9,[]);
      [x_rpy_cylinder,dx_rpy_cylinder] = rotmat2rpy(x_rotmat_cylinder,dx_rotmat_cylinder); 
      x_cylinder(4:6) = x_rpy_cylinder;
      J(4:6,:) = dx_rpy_cylinder;
    end
  end
end