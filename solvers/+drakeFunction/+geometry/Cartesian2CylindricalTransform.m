classdef Cartesian2CylindricalTransform < drakeFunction.DrakeFunction
  % Given a cartesian coordinate frame, transform its coordinates to a
  % cylindrical coordinates
  properties
    T_cylinder % This is a homogeneous transformation matrix, T_cylinder transforms a cylinder with [0;0;1] being its axis, and [1;0;0] as its theta=0 radius, to the cylinder in the cartesian frame
  end
  
  methods
    function obj = Cartesian2CylindricalTransform(T_cylinder)
      input_frame = drakeFunction.frames.realCoordinateSpace(6);
      output_frame = drakeFunction.frames.realCoordinateSpace(6);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      if(~isHT(T_cylinder))
        error('T_cylinder should be a homogenous transformation matrix');
      end
      obj.T_cylinder = T_cylinder;
    end
    
    function [x_cylinder,J] = eval(obj,x_cartesian)
      % x_cylinder = [radius;theta;height;roll;pitch;yaw]
      % we attach a cartesian frame [x_tangent;y_normal;z_tangent] in R^3x3 at the tip of the
      % radius, where x_tangent is aligned with cross(height,radius),
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