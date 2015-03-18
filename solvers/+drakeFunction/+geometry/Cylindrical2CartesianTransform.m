classdef Cylindrical2CartesianTransform < drakeFunction.DrakeFunction
  % Given a cylindrical coordinate frame, transform its coordinates to a
  % Cartesian coordinate. Refer to drake/doc/doc_cartesian2cylindrical.pdf for the
  % transformation
  properties(SetAccess = protected)
    cylinder_axis % A 3 x 1 vector, the axis of the cylinder in the Cartesian coordinate
    cylinder_x_dir % A 3 x 1 vector, the cylinder x direction (the direction that theta=0 points to) in the Cartesian coordinate
    cylinder_origin % A 3 x 1 vector, the origin (height = 0) point of the cylinder in the Cartesian coordinate. It should be perpendicular to cylinder_axis.
    T_cylinder % This is a homogeneous transformation matrix, T_cylinder transforms a cylinder with [0;0;1] being its axis, and nd the angle theta measured with respect to the x-axis
  end
  
  methods
    function obj = Cylindrical2CartesianTransform(cylinder_axis,cylinder_x_dir,cylinder_origin)
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
    
    function [x_cartesian,J,dJ] = eval(obj,x_cylinder)
      % Refer to drake/doc/doc_cartesian2cylindrical.pdf for the transformation
      % @retval x_cartesian = [x;y;z;roll;pitch;yaw];
      % @param x_cylinder =
      % [radius;theta;height;roll_tangent,pitch_tangent,yaw_tangent]
      % @retval J  A 6 x 6 matrix, the Jacobian of x_cartesian w.r.t x_cylinder
      % @retval dJ A 6 x 36 matrix, the second gradient of x_cartesian
      % w.r.t x_cylinder, in the geval format
      radius = x_cylinder(1);
      theta = x_cylinder(2);
      height = x_cylinder(3);
      c_theta = cos(theta);
      s_theta = sin(theta);
      x_pos_cartesian = obj.T_cylinder(1:3,1:3)*[radius*c_theta;radius*s_theta;height]+obj.T_cylinder(1:3,4);
      dx_pos_cartesian = obj.T_cylinder(1:3,1:3)*[[c_theta radius*-s_theta 0;s_theta radius*c_theta 0;0 0 1] zeros(3,3)];
      ddx_idx = reshape(1:36,6,6);
      ddx_pos_idx = reshape(ddx_idx(1:3,1:3),[],1);
      ddx_rpy_idx = reshape(ddx_idx(4:6,4:6),[],1);
      ddx_pos_cartesian = zeros(3,36);
      ddx_pos_cartesian(:,ddx_pos_idx) = obj.T_cylinder(1:3,1:3)*[[0;0;0] [-s_theta;c_theta;0] zeros(3,1) [-s_theta;c_theta;0] [-radius*c_theta;-radius*s_theta;0] zeros(3,1) zeros(3,3)];
      [R_tangent,dR_tangent,ddR_tangent_ddx_rpy_cylinder] = rpy2rotmat(x_cylinder(4:6));
      dR_tangent = [zeros(9,3) dR_tangent];
      ddR_tangent = zeros(9,36);
      ddR_tangent(:,ddx_rpy_idx) = ddR_tangent_ddx_rpy_cylinder;
      [R_tangent2cylinder,dR_tangent2cylinder,ddR_tangent2cylinder_ddtheta] = rotz(pi/2-theta);
      dR_tangent2cylinder = [zeros(9,1) -dR_tangent2cylinder(:) zeros(9,4)];
      ddR_tangent2cylinder = zeros(9,36);
      ddR_tangent2cylinder(:,8) = ddR_tangent2cylinder_ddtheta(:);
      R_cylinder = R_tangent2cylinder*R_tangent;
      dR_cylinder = matGradMultMat(R_tangent2cylinder,R_tangent,dR_tangent2cylinder,dR_tangent);
      ddR_cylinder = matHessianMultMat(R_tangent2cylinder,R_tangent,dR_tangent2cylinder,dR_tangent,ddR_tangent2cylinder,ddR_tangent);
      [x_rpy_cartesian,dx_rpy_cartesian] = rotmat2rpy(obj.T_cylinder(1:3,1:3)*R_cylinder,reshape(obj.T_cylinder(1:3,1:3)*reshape(dR_cylinder,3,[]),9,[]));
      x_cartesian = [x_pos_cartesian];%x_rpy_cartesian];
      J = [dx_pos_cartesian];%;dx_rpy_cartesian];
      dJ = ddx_pos_cartesian;
      
      
    end
  end
end